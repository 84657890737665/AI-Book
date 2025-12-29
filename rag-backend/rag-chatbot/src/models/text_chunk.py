"""TextChunk model for the RAG Pipeline."""
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional
from .base import BaseModel


@dataclass(kw_only=True)
class TextChunk(BaseModel):
    """Represents a segment of processed text that has been cleaned, deduplicated, and optimally chunked for RAG processing."""
    
    content_id: str
    text: str
    source_url: str
    page_title: str
    chunk_index: int = 0
    token_count: Optional[int] = None
    
    def __post_init__(self):
        """Validate the TextChunk after initialization."""
        if not self.content_id:
            raise ValueError("Content ID must not be empty")
        
        if not self.text:
            raise ValueError("Text must not be empty")
        
        if not self.source_url:
            raise ValueError("Source URL must not be empty")
        
        if not self.page_title:
            raise ValueError("Page title must not be empty")
        
        if self.chunk_index < 0:
            raise ValueError("Chunk index must be non-negative")
        
        if self.token_count is not None and self.token_count <= 0:
            raise ValueError("Token count must be positive if specified")
    
    def calculate_token_count(self) -> int:
        """Calculate the number of tokens in the text (simple approximation)."""
        # This is a simple approximation; in practice, you might use a proper tokenizer
        self.token_count = len(self.text.split())
        return self.token_count