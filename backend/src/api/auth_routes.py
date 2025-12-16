from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from jose import JWTError, jwt
from datetime import datetime, timedelta
from typing import Optional
import secrets
import hashlib

from src.services.authentication_service import (
    authenticate_user,
    create_access_token,
    store_session_token,
    invalidate_session
)
from config.settings import settings
from src.models.user_session import UserSession
from config.database import SessionLocal

router = APIRouter()
security = HTTPBearer()


@router.post("/login")
async def login(username: str, password: str):
    """
    Authenticate user and return access token.
    """
    user = authenticate_user(username, password)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Create access token
    access_token_expires = timedelta(minutes=settings.access_token_expire_minutes)
    access_token = create_access_token(
        data={"sub": user["username"]},
        expires_delta=access_token_expires
    )

    # Store session in database
    expires_at = datetime.utcnow() + access_token_expires
    store_session_token(
        user_id=user["username"],
        token=access_token,
        expires_at=expires_at
    )

    return {
        "access_token": access_token,
        "token_type": "bearer",
        "expires_in": access_token_expires.seconds
    }


@router.post("/logout")
async def logout(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """
    Invalidate user session.
    """
    token = credentials.credentials

    # Invalidate the session in the database
    success = invalidate_session(token)

    if not success:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid or expired token"
        )

    return {"message": "Successfully logged out"}


def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)) -> Optional[str]:
    """
    Get current user from token.
    """
    token = credentials.credentials
    try:
        # Verify token
        payload = jwt.decode(token, settings.secret_key, algorithms=[settings.algorithm])
        username: str = payload.get("sub")
        if username is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Check if token is still active in database
        db = SessionLocal()
        try:
            session = db.query(UserSession).filter(
                UserSession.session_token == token,
                UserSession.active == True
            ).first()

            if session is None or session.expires_at < datetime.utcnow():
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Token has expired or is invalid",
                    headers={"WWW-Authenticate": "Bearer"},
                )

            return username
        finally:
            db.close()

    except JWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )