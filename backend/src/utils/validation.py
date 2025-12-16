from typing import Optional
from pydantic import BaseModel, validator
import re


def validate_book_title(title: str) -> str:
    """Validate book title."""
    if not title or len(title.strip()) == 0:
        raise ValueError("Title cannot be empty")
    
    if len(title) > 500:
        raise ValueError("Title exceeds maximum length of 500 characters")
    
    return title.strip()


def validate_author_name(author: str) -> str:
    """Validate author name."""
    if not author or len(author.strip()) == 0:
        raise ValueError("Author name cannot be empty")
    
    if len(author) > 200:
        raise ValueError("Author name exceeds maximum length of 200 characters")
    
    return author.strip()


def validate_query_text(query: str) -> str:
    """Validate user query text."""
    if not query or len(query.strip()) == 0:
        raise ValueError("Query cannot be empty")
    
    if len(query) > 2000:
        raise ValueError("Query exceeds maximum length of 2000 characters")
    
    return query.strip()


def validate_selected_text(selected_text: Optional[str]) -> Optional[str]:
    """Validate selected text."""
    if selected_text is None:
        return None
    
    if len(selected_text) > 5000:
        raise ValueError("Selected text exceeds maximum length of 5000 characters")
    
    return selected_text


def validate_file_extension(filename: str) -> bool:
    """Validate file extension is allowed."""
    allowed_extensions = {".pdf", ".epub", ".txt"}
    ext = "." + filename.split(".")[-1].lower()
    return ext in allowed_extensions


def sanitize_user_input(input_str: str) -> str:
    """Sanitize user input to prevent injection attacks."""
    # Remove potential SQL injection characters
    sanitized = re.sub(r"[;'\"]", "", input_str)
    # Remove potential script tags
    sanitized = re.sub(r"<script[^>]*>.*?</script>", "", sanitized, flags=re.IGNORECASE)
    return sanitized.strip()


class QueryValidationModel(BaseModel):
    query: str
    mode: str = "full_book"
    selected_text: Optional[str] = None
    
    @validator("query")
    def validate_query(cls, v):
        return validate_query_text(v)
    
    @validator("mode")
    def validate_mode(cls, v):
        if v not in ["full_book", "selected_text"]:
            raise ValueError("Mode must be either 'full_book' or 'selected_text'")
        return v
    
    @validator("selected_text")
    def validate_selected_text_field(cls, v):
        return validate_selected_text(v)