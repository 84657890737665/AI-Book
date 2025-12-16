from sqlalchemy import Column, Integer, String, DateTime, Text, Enum, ForeignKey
from sqlalchemy.dialects.postgresql import UUID as PG_UUID, JSONB
from config.database import Base
import uuid
from datetime import datetime
from enum import Enum as PyEnum


class MultimediaType(PyEnum):
    IMAGE = "IMAGE"
    TABLE = "TABLE"
    DIAGRAM = "DIAGRAM"
    AUDIO = "AUDIO"
    VIDEO = "VIDEO"


class MultimediaContent(Base):
    __tablename__ = "multimedia_content"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    book_id = Column(PG_UUID(as_uuid=True), ForeignKey("book_content.id"), nullable=False)
    content_type = Column(Enum(MultimediaType), nullable=False)
    content_path = Column(String, nullable=False)  # Path to the multimedia file
    text_description = Column(Text, nullable=False)  # Text description of multimedia content
    alt_text = Column(Text, nullable=True)  # Alternative text for accessibility
    metadata = Column(JSONB, nullable=True)  # Additional metadata about the multimedia
    created_at = Column(DateTime, default=datetime.utcnow)

    def __repr__(self):
        return f"<MultimediaContent(content_type='{self.content_type}', book_id='{self.book_id}')>"