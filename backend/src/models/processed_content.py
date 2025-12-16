from sqlalchemy import Column, Integer, String, DateTime, Text, ForeignKey
from sqlalchemy.dialects.postgresql import UUID as PG_UUID, JSONB
from config.database import Base
import uuid
from datetime import datetime


class ProcessedContent(Base):
    __tablename__ = "processed_content"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    book_id = Column(PG_UUID(as_uuid=True), ForeignKey("book_content.id"), nullable=False)
    chunk_id = Column(String, nullable=False)  # Unique identifier for the chunk in the vector store
    chunk_text = Column(Text, nullable=False)  # The actual text content of the chunk
    chunk_metadata = Column(JSONB, nullable=False)  # Metadata about the chunk (page number, section, etc.)
    embedding_id = Column(String, nullable=False)  # ID of the embedding in the vector store
    created_at = Column(DateTime, default=datetime.utcnow)

    def __repr__(self):
        return f"<ProcessedContent(book_id='{self.book_id}', chunk_id='{self.chunk_id}')>"