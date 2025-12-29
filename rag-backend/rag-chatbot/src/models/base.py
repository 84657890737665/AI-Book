"""Base model classes for the RAG Pipeline."""
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, Any, Optional


@dataclass(kw_only=True)
class BaseModel:
    """Base class for all models in the RAG Pipeline."""
    id: str
    created_at: datetime = field(default_factory=datetime.now)
    updated_at: datetime = field(default_factory=datetime.now)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert the model to a dictionary representation."""
        result = {}
        for key, value in self.__dict__.items():
            if isinstance(value, datetime):
                result[key] = value.isoformat()
            else:
                result[key] = value
        return result
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]):
        """Create an instance from a dictionary representation."""
        # This is a basic implementation - subclasses may need to override
        # to handle datetime conversion and other special cases
        instance = cls.__new__(cls)
        for key, value in data.items():
            if hasattr(instance, key):
                setattr(instance, key, value)
        return instance