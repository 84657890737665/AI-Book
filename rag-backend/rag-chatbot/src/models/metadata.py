"""VectorMetadata model for the RAG Pipeline."""
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, Any, Optional
from .base import BaseModel


@dataclass(kw_only=True)
class VectorMetadata(BaseModel):
    """Represents additional information stored with each embedding vector."""
    
    vector_id: str
    source_url: str
    section_title: str
    chunk_identifier: str
    chunk_text: str  # The actual text content of the chunk
    additional_metadata: Optional[Dict[str, Any]] = field(default_factory=dict)
    
    def __post_init__(self):
        """Validate the VectorMetadata after initialization."""
        if not self.vector_id:
            raise ValueError("Vector ID must not be empty")
        
        if not self.source_url:
            raise ValueError("Source URL must not be empty")
        
        if not self.section_title:
            raise ValueError("Section title must not be empty")
        
        if not self.chunk_identifier:
            raise ValueError("Chunk identifier must not be empty")
        
        if not self.chunk_text:
            raise ValueError("Chunk text must not be empty")
    
    def add_metadata(self, key: str, value: Any) -> None:
        """Add additional metadata to the metadata dictionary."""
        self.additional_metadata[key] = value
        self.updated_at = datetime.now()