"""Error handling utilities for the RAG Pipeline."""
from typing import Optional


class RAGPipelineError(Exception):
    """Base exception class for RAG Pipeline errors."""
    pass


class URLFetchError(RAGPipelineError):
    """Raised when there's an error fetching URLs."""
    pass


class TextExtractionError(RAGPipelineError):
    """Raised when there's an error extracting text from content."""
    pass


class EmbeddingGenerationError(RAGPipelineError):
    """Raised when there's an error generating embeddings."""
    pass


class VectorStorageError(RAGPipelineError):
    """Raised when there's an error storing vectors."""
    pass


class ProcessingStateError(RAGPipelineError):
    """Raised when there's an error managing processing state."""
    pass


def handle_error(error: Exception, context: str = "", logger=None) -> None:
    """
    Generic error handling function.
    
    Args:
        error: The exception that occurred
        context: Additional context about where the error occurred
        logger: Optional logger instance to log the error
    """
    error_msg = f"Error in {context}: {str(error)} (Type: {type(error).__name__})"
    
    if logger:
        logger.error(error_msg, exc_info=True)
    else:
        print(error_msg)