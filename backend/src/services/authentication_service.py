from datetime import datetime, timedelta
from typing import Optional
from jose import JWTError, jwt
from passlib.context import CryptContext
from config.settings import settings
from src.models.user_session import UserSession
from config.database import SessionLocal


# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a plain password against its hash."""
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password: str) -> str:
    """Hash a plain password."""
    return pwd_context.hash(password)


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """Create a JWT access token."""
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=settings.access_token_expire_minutes)
    
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, settings.secret_key, algorithm=settings.algorithm)
    return encoded_jwt


def authenticate_user(username: str, password: str):
    """Authenticate a user by username and password."""
    # In a real implementation, this would query the database for a user
    # For the demo, we'll return a mock user if credentials match
    if username == "demo@example.com" and password == "DemoPassword123!":
        return {"username": username, "email": username}
    return None


def store_session_token(user_id: str, token: str, expires_at: datetime):
    """Store the session token in the database."""
    db = SessionLocal()
    try:
        session = UserSession(
            user_id=user_id,
            session_token=token,
            expires_at=expires_at
        )
        db.add(session)
        db.commit()
        db.refresh(session)
        return session
    finally:
        db.close()


def invalidate_session(token: str):
    """Invalidate a session by setting its active status to False."""
    db = SessionLocal()
    try:
        session = db.query(UserSession).filter(UserSession.session_token == token).first()
        if session:
            session.active = False
            db.commit()
            return True
        return False
    finally:
        db.close()