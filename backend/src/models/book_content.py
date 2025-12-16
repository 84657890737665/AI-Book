from sqlalchemy import Column, Integer, String, DateTime, Text, Enum, UUID
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from sqlalchemy.orm import relationship
from config.database import Base
import uuid
from datetime import datetime
from enum import Enum as PyEnum


class ProcessingStatus(PyEnum):
    PENDING = "PENDING"
    PROCESSING = "PROCESSING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


class BookContentType(PyEnum):
    PDF = "PDF"
    EPUB = "EPUB"
    TEXT = "TEXT"
    OTHER = "OTHER"


class BookContent(Base):
    __tablename__ = "book_content"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    title = Column(String(500), nullable=False)
    author = Column(String(200), nullable=False)
    isbn = Column(String(20), nullable=True)
    content_type = Column(Enum(BookContentType), nullable=False)
    content_path = Column(String, nullable=False)
    total_pages = Column(Integer, nullable=True)
    language = Column(String(50), nullable=False)
    processing_status = Column(Enum(ProcessingStatus), default=ProcessingStatus.PENDING)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    # Relationship to UserQuery
    queries = relationship("UserQuery", order_by="UserQuery.created_at", back_populates="book")

    def __repr__(self):
        return f"<BookContent(title='{self.title}', author='{self.author}')>"