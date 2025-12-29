"""
Hugging Face Spaces application for the RAG Pipeline API.
Also acts as a CLI entry point for data ingestion.

Usage:
  Run Server:    python app.py
  Run Ingestion: python app.py --urls "https://example.com"
"""
import sys
import argparse
import asyncio
import uvicorn
from src.api.main import app
from src.services.url_fetcher import URLFetcher
from src.services.text_extractor import TextExtractor
from src.services.text_chunker import TextChunker
from src.services.embedding_generator import EmbeddingGenerator
from src.services.vector_storage import VectorStorage
from src.models.metadata import VectorMetadata

async def run_ingestion(url: str, chunk_size: int = 1000, chunk_overlap: int = 200):
    """Run the ingestion pipeline for a given URL."""
    print(f"Starting ingestion pipeline for: {url}")
    
    try:
        # Phase 1: URL Fetching
        print("Phase 1: Fetching URLs...")
        url_fetcher = URLFetcher(url)
        urls = url_fetcher.fetch_urls(include_subpages=True)
        print(f"Found {len(urls)} URLs.")

        # Phase 2: Content Extraction
        print("Phase 2: Extracting content...")
        text_extractor = TextExtractor()
        book_contents = text_extractor.extract_content(urls)
        deduplicated_contents = text_extractor.deduplicate_content(book_contents)
        print(f"Extracted content from {len(deduplicated_contents)} unique pages.")

        # Prepare for chunking
        content_items = []
        for content in deduplicated_contents:
            content_items.append({
                "id": content.id,
                "url": content.url,
                "title": content.title,
                "text": content.clean_text
            })

        # Phase 3: Chunking
        print("Phase 3: Chunking text...")
        text_chunker = TextChunker(chunk_size=chunk_size, chunk_overlap=chunk_overlap)
        text_chunks = text_chunker.chunk_content(content_items)
        print(f"Created {len(text_chunks)} chunks.")

        # Phase 4: Embeddings
        print("Phase 4: Generating embeddings (this may take a while)...")
        embedding_generator = EmbeddingGenerator()
        
        chunks_for_embedding = []
        for chunk in text_chunks:
            chunks_for_embedding.append({
                "id": chunk.id,
                "text": chunk.text
            })
            
        embeddings = embedding_generator.generate_embeddings(chunks_for_embedding)
        print(f"Generated {len(embeddings)} embeddings.")

        # Phase 5: Storage
        print("Phase 5: Storing using Qdrant...")
        vector_storage = VectorStorage()
        
        metadata_list = []
        for i, embedding in enumerate(embeddings):
            chunk = text_chunks[i] if i < len(text_chunks) else None
            source_url = chunk.content_id if chunk else "unknown"
            
            metadata = VectorMetadata(
                id=f"meta_{embedding.id}",
                vector_id=embedding.id,
                source_url=source_url, 
                section_title=chunk.text[:50] + "..." if chunk else "unknown",
                chunk_identifier=embedding.chunk_id,
            )
            metadata_list.append(metadata)

        vector_storage.store_embeddings(embeddings, metadata_list)
        print("Ingestion complete! Data stored in Qdrant.")

    except Exception as e:
        print(f"Error during ingestion: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    # Check if arguments are provided
    parser = argparse.ArgumentParser(description="RAG Pipeline Application")
    parser.add_argument("--urls", help="The URL to ingest content from (runs ingestion mode)")
    parser.add_argument("--chunk-size", type=int, default=1000, help="Size of text chunks")
    parser.add_argument("--chunk-overlap", type=int, default=200, help="Overlap between chunks")
    
    # We use parse_known_args to avoid conflict if uvicorn args are passed (though usually purely separated)
    args, unknown = parser.parse_known_args()

    if args.urls:
        # Run Ingestion Mode
        asyncio.run(run_ingestion(args.urls, args.chunk_size, args.chunk_overlap))
    else:
        # Run Server Mode
        print("Starting RAG Chatbot API Server...")
        print("Access the API at http://localhost:8000")
        uvicorn.run(app, host="0.0.0.0", port=8000)