from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # Application settings
    app_name: str = "Integrated RAG Chatbot API"
    admin_email: str = "admin@rag-chatbot.example.com"
    version: str = "1.0.0"

    # Cohere settings
    cohere_api_key: str
    cohere_model_generation: str = "command-r-plus"
    cohere_model_embedding: str = "embed-multilingual-v3.0"

    # Database settings
    database_url: str

    # Qdrant settings
    qdrant_host: str
    qdrant_api_key: str
    qdrant_cluster_id: str

    # Target site for crawling (for embedding pipeline)
    target_site_url: str = "https://docs.python.org/3/"

    # Security settings
    secret_key: str
    algorithm: str = "HS256"
    access_token_expire_minutes: int = 30

    # Upload settings
    max_upload_size: int = 50 * 1024 * 1024  # 50 MB
    allowed_extensions: set = {"pdf", "epub", "txt"}

    # Performance settings
    response_timeout: int = 30
    max_query_length: int = 2000
    max_selected_text_length: int = 5000

    class Config:
        env_file = ".env"


settings = Settings()