import sys
import argparse
import asyncio
from src.services.url_fetcher import URLFetcher
from src.services.text_extractor import TextExtractor
from src.services.text_chunker import TextChunker
from src.services.embedding_generator import EmbeddingGenerator
from src.services.vector_storage import VectorStorage
from src.lib.logging import logger
from src.models.metadata import VectorMetadata

async def main():
    parser = argparse.ArgumentParser(description="Ingest content from a URL into the RAG pipeline.")
    parser.add_argument("--url", required=True, help="The URL to ingest content from")
    parser.add_argument("--chunk-size", type=int, default=1000, help="Size of text chunks")
    parser.add_argument("--chunk-overlap", type=int, default=200, help="Overlap between chunks")
    args = parser.parse_args()

    print(f"Starting ingestion pipeline for: {args.url}")
    
    try:
        # Phase 1: URL Fetching
        print("Phase 1: Fetching URLs...")
        url_fetcher = URLFetcher(args.url)
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
        text_chunker = TextChunker(chunk_size=args.chunk_size, chunk_overlap=args.chunk_overlap)
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
            
            if not chunk:
                continue
            
            metadata = VectorMetadata(
                id=f"meta_{embedding.id}",
                vector_id=embedding.id,
                source_url=chunk.source_url,
                section_title=chunk.page_title,
                chunk_identifier=embedding.chunk_id,
                chunk_text=chunk.text  # Pass the actual chunk content
            )
            metadata_list.append(metadata)

        vector_storage.store_embeddings(embeddings, metadata_list)
        print("Ingestion complete! Data stored in Qdrant.")

    except Exception as e:
        print(f"Error during ingestion: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(main())
