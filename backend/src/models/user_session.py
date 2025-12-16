from sqlalchemy import Column, String, DateTime, Boolean
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from config.database import Base
import uuid
from datetime import datetime


class UserSession(Base):
    __tablename__ = "user_sessions"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(String, nullable=False)  # Unique identifier for the user
    session_token = Column(String, nullable=False, unique=True)  # JWT token for the session
    created_at = Column(DateTime, default=datetime.utcnow)
    expires_at = Column(DateTime, nullable=False)  # Timestamp when the session expires
    active = Column(Boolean, nullable=False, default=True)  # Whether the session is still active

    def __repr__(self):
        return f"<UserSession(user_id='{self.user_id}', active={self.active})>"