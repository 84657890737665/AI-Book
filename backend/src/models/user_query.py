from sqlalchemy import Column, Integer, String, DateTime, Text, Enum, UUID, ForeignKey
from sqlalchemy.dialects.postgresql import UUID as PG_UUID, JSONB
from sqlalchemy.orm import relationship
from config.database import Base
import uuid
from datetime import datetime
from enum import Enum as PyEnum


class QueryMode(PyEnum):
    FULL_BOOK = "FULL_BOOK"
    SELECTED_TEXT = "SELECTED_TEXT"


class UserQuery(Base):
    __tablename__ = "user_queries"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(String, nullable=False)  # Using string for simplicity instead of UUID
    book_id = Column(PG_UUID(as_uuid=True), ForeignKey("book_content.id"), nullable=False)
    query_text = Column(Text, nullable=False)
    query_mode = Column(Enum(QueryMode), default=QueryMode.FULL_BOOK)
    selected_text = Column(Text, nullable=True)
    retrieved_context = Column(JSONB, nullable=False)
    response_text = Column(Text, nullable=False)
    citation_references = Column(JSONB, nullable=False)
    query_latency = Column(Integer, nullable=True)  # Latency in milliseconds
    created_at = Column(DateTime, default=datetime.utcnow)

    # Relationship to BookContent
    book = relationship("BookContent", back_populates="queries")

    def __repr__(self):
        return f"<UserQuery(user_id='{self.user_id}', book_id='{self.book_id}', mode='{self.query_mode}')>"


