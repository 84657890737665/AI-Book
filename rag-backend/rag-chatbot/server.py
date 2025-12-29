"""Server entry point for the RAG Pipeline API."""
import uvicorn
import os
from src.api.main import app


if __name__ == "__main__":
    port = int(os.getenv("PORT", 8000))
    host = os.getenv("HOST", "0.0.0.0")
    
    uvicorn.run(
        "src.api.main:app",
        host=host,
        port=port,
        reload=True,  # Set to False in production
        log_level="info"
    )