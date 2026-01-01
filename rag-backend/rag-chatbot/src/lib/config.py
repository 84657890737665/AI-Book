"""Configuration management for the RAG Pipeline."""
import os
import re
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Config:
    """Configuration class to manage environment variables and settings."""

    # Cohere API configuration
    COHERE_API_KEY = os.getenv("COHERE_API_KEY")
    COHERE_MODEL = os.getenv("COHERE_MODEL", "embed-multilingual-v2.0")  # Default to multilingual model

    # Qdrant configuration
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    QDRANT_URL = os.getenv("QDRANT_URL")

    # Processing configuration
    CHUNK_SIZE = int(os.getenv("CHUNK_SIZE", "1000"))
    CHUNK_OVERLAP = int(os.getenv("CHUNK_OVERLAP", "200"))
    BATCH_SIZE = int(os.getenv("BATCH_SIZE", "10"))

    # Security configuration
    ALLOWED_HOSTS = os.getenv("ALLOWED_HOSTS", "").split(",")
    ALLOWED_ORIGINS = os.getenv("ALLOWED_ORIGINS", "*").split(",")  # Frontend origins for CORS
    MAX_FILE_SIZE = int(os.getenv("MAX_FILE_SIZE", "5000000"))  # 5MB in bytes
    SECURE_SSL_REDIRECT = os.getenv("SECURE_SSL_REDIRECT", "True").lower() == "true"

    # Directories
    OUTPUT_DIR = os.getenv("OUTPUT_DIR", "./output")

    @classmethod
    def validate(cls):
        """Validate that required configuration values are present."""
        required_vars = ["COHERE_API_KEY", "QDRANT_API_KEY", "QDRANT_URL"]
        missing_vars = [var for var in required_vars if not getattr(cls, var)]

        if missing_vars:
            error_msg = f"Missing required environment variables: {', '.join(missing_vars)}"
            if os.getenv("SPACE_ID"):  # Detect Hugging Face Spaces environment
                error_msg += ". Please add these as Secrets in your Hugging Face Space Settings."
            raise ValueError(error_msg)

    @classmethod
    def validate_api_key_format(cls):
        """Validate that API keys have appropriate formats."""
        # Validate Cohere API key format (typically starts with 'cohere-' or is a UUID-like string)
        if cls.COHERE_API_KEY:
            # Allow alphanumeric, dashes, dots, underscores. Min length 20 to be safe.
            if not re.match(r'^[a-zA-Z0-9._-]{20,}$', cls.COHERE_API_KEY):
                raise ValueError(f"Cohere API key format is invalid: {cls.COHERE_API_KEY[:4]}...")

        # Validate Qdrant API key format (typically a UUID-like string)
        if cls.QDRANT_API_KEY:
            # Allow alphanumeric, dashes, dots, underscores. Min length 20 to be safe.
            if not re.match(r'^[a-zA-Z0-9._-]{20,}$', cls.QDRANT_API_KEY):
                raise ValueError(f"Qdrant API key format is invalid: {cls.QDRANT_API_KEY[:4]}...")

    @classmethod
    def sanitize_url(cls, url: str) -> str:
        """Sanitize and validate URLs to prevent security issues."""
        # Check if URL has a valid format
        if not re.match(r'^https?://[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}(/.*)?$', url):
            raise ValueError(f"Invalid URL format: {url}")

        # Check if URL is in allowed hosts (if specified)
        if cls.ALLOWED_HOSTS and cls.ALLOWED_HOSTS != [""]:
            from urllib.parse import urlparse
            parsed = urlparse(url)
            if parsed.netloc not in cls.ALLOWED_HOSTS:
                raise ValueError(f"URL host not in allowed hosts: {parsed.netloc}")

        return url