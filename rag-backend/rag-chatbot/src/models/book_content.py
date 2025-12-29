"""BookContent model for the RAG Pipeline."""
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional
from .base import BaseModel


@dataclass(kw_only=True)
class BookContent(BaseModel):
    """Represents the textual content extracted from deployed book websites."""

    url: str
    title: str
    raw_text: str
    clean_text: Optional[str] = None

    def __post_init__(self):
        """Validate the BookContent after initialization."""
        if not self.url:
            raise ValueError("URL must not be empty")

        if not self.raw_text:
            raise ValueError("Raw text must not be empty")

    def update_content(self, clean_text: str):
        """Update the clean text content and update the timestamp."""
        self.clean_text = clean_text
        self.updated_at = datetime.now()