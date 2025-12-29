"""Quickstart validation script for the RAG Pipeline."""
import sys
import os
from pathlib import Path

# Add the src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def validate_quickstart():
    """Validate that the quickstart instructions work as expected."""
    print("Validating quickstart instructions...")

    # Test 1: Check that required dependencies can be imported
    try:
        import requests
        import bs4
        import langchain
        import cohere
        import qdrant_client
        import dotenv
        print("[OK] All required dependencies can be imported")
    except ImportError as e:
        print(f"[ERROR] Dependency import failed: {e}")
        return False

    # Test 2: Check that main CLI module can be imported
    try:
        from src.cli.main import main
        print("[OK] Main CLI module can be imported")
    except ImportError as e:
        print(f"[ERROR] CLI module import failed: {e}")
        return False

    # Test 3: Check that configuration can be loaded (without actual API keys)
    try:
        from src.lib.config import Config
        # We won't call validate() since it requires API keys
        print("[OK] Configuration module can be imported")
    except ImportError as e:
        print(f"[ERROR] Configuration module import failed: {e}")
        return False

    # Test 4: Check that all core services can be imported
    try:
        from src.services.url_fetcher import URLFetcher
        from src.services.text_extractor import TextExtractor
        from src.services.text_chunker import TextChunker
        from src.services.embedding_generator import EmbeddingGenerator
        from src.services.vector_storage import VectorStorage
        print("[OK] All core service modules can be imported")
    except ImportError as e:
        print(f"[ERROR] Service module import failed: {e}")
        return False

    # Test 5: Check that all core models can be imported
    try:
        from src.models.book_content import BookContent
        from src.models.text_chunk import TextChunk
        from src.models.embedding_vector import EmbeddingVector
        from src.models.metadata import VectorMetadata
        print("[OK] All core model modules can be imported")
    except ImportError as e:
        print(f"[ERROR] Model module import failed: {e}")
        return False

    print("\n[SUCCESS] Quickstart validation completed successfully!")
    print("The RAG Pipeline project structure is correctly set up.")
    print("To run the pipeline, you will need valid API keys for Cohere and Qdrant.")

    return True

if __name__ == "__main__":
    success = validate_quickstart()
    if not success:
        sys.exit(1)
    else:
        print("\nYou can now run the pipeline with a command like:")
        print("python -m src.cli.main --url \"https://your-book-url.com\" --process-all")