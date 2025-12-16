import os
import logging
from typing import List
from dotenv import load_dotenv

import requests
from bs4 import BeautifulSoup
from langchain_text_splitters import RecursiveCharacterTextSplitter


import cohere
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, VectorParams, Distance

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class DocusaurusEmbeddingPipeline:
    def __init__(self):
        # Initialize Cohere client
        cohere_api_key = os.getenv("ml0YZj88cPvNZbxW1ysKxMxbJTlcwOA5BcGeGjKO")
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY not found in environment variables")
        self.cohere_client = cohere.Client(api_key=cohere_api_key)

        # Initialize Qdrant client
        qdrant_url = os.getenv("https://2e580569-3a35-4d4c-bfc2-71189db2e78d.europe-west3-0.gcp.cloud.qdrant.io:6333")
        qdrant_api_key = os.getenv("eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.nLimUBljSFhYj8iztY5KZYJ5C5WgzAhi--kljHYJTOc")

        if qdrant_api_key:
            self.qdrant_client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                timeout=60,
            )
        else:
            self.qdrant_client = QdrantClient(url=qdrant_url, timeout=60)

        logger.info("Cohere and Qdrant clients initialized successfully.")

        # Target Docusaurus site URL (update as needed)
        self.target_url = os.getenv(
            "TARGET_SITE_URL",
            "https://hackathon-physical-ai-humanoid-robotics.netlify.app/"  # Example or your actual site
        )

        # Qdrant collection settings
        self.collection_name = os.getenv("QDRANT_COLLECTION", "physical_ai_book")
        self.vector_size = 1024  # Cohere embed-multilingual-v3.0 dimension

    def fetch_page_text(self, url: str) -> str:
        """Fetch and extract clean text from a webpage."""
        try:
            response = requests.get(url, timeout=10)
            response.raise_for_status()
            soup = BeautifulSoup(response.text, "html.parser")

            # Remove scripts, styles, nav, footer, etc.
            for element in soup(["script", "style", "nav", "footer", "header", "aside"]):
                element.decompose()

            text = soup.get_text(separator=" ", strip=True)
            return text
        except Exception as e:
            logger.error(f"Failed to fetch {url}: {e}")
            return ""

    def crawl_and_extract(self, base_url: str) -> List[str]:
        """Simple crawler to extract text from main pages (extend for full crawl if needed)."""
        # For a Docusaurus site, common pages:
        urls = [
            base_url,
            base_url.rstrip("/") + "/docs/intro",
            base_url.rstrip("/") + "/blog",
            # Add more known paths or implement proper link discovery
        ]

        all_texts = []
        for url in urls:
            logger.info(f"Extracting text from: {url}")
            text = self.fetch_page_text(url)
            if text:
                all_texts.append({"source": url, "content": text})

        return all_texts

    def chunk_text(self, documents: List[dict]) -> List[dict]:
        """Chunk documents into smaller pieces with metadata."""
        text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=1000,
            chunk_overlap=200,
            length_function=len,
            separators=["\n\n", "\n", " ", ""]
        )

        chunks = []
        for doc_idx, doc in enumerate(documents):
            splits = text_splitter.split_text(doc["content"])
            for i, chunk in enumerate(splits):
                chunks.append({
                    "id": str(abs(hash(f"{doc['source']}_{doc_idx}_{i}")))[:10],  # Use string ID with limited length
                    "text": chunk,
                    "metadata": {
                        "source": doc["source"],
                        "chunk_index": i
                    }
                })
        logger.info(f"Created {len(chunks)} chunks.")
        return chunks

    def embed_chunks(self, chunks: List[dict]) -> List[dict]:
        """Generate embeddings using Cohere."""
        texts = [chunk["text"] for chunk in chunks]
        logger.info("Generating embeddings with Cohere...")

        response = self.cohere_client.embed(
            texts=texts,
            model="embed-multilingual-v3.0",
            input_type="search_document"
        )

        for i, chunk in enumerate(chunks):
            chunk["vector"] = response.embeddings[i]

        logger.info("Embeddings generated successfully.")
        return chunks

    def create_collection(self):
        """Create Qdrant collection if it doesn't exist."""
        collections = self.qdrant_client.get_collections()
        if self.collection_name not in [c.name for c in collections.collections]:
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.vector_size,
                    distance=Distance.COSINE
                )
            )
            logger.info(f"Created collection: {self.collection_name}")
        else:
            logger.info(f"Collection {self.collection_name} already exists.")

    def upload_to_qdrant(self, embedded_chunks: List[dict]):
        """Upload embedded chunks as points to Qdrant."""
        points = []
        for chunk in embedded_chunks:
            # Convert string ID to a valid Qdrant ID (UUID string or unsigned 32-bit integer)
            # Using UUID string format for large or complex IDs
            import uuid
            # Generate a UUID based on the original ID string to ensure consistency
            point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, chunk["id"]))

            points.append(
                PointStruct(
                    id=point_id,
                    vector=chunk["vector"],
                    payload={
                        "text": chunk["text"],
                        "source": chunk["metadata"]["source"],
                        "chunk_index": chunk["metadata"]["chunk_index"]
                    }
                )
            )

        self.qdrant_client.upsert(
            collection_name=self.collection_name,
            points=points
        )
        logger.info(f"Uploaded {len(points)} points to Qdrant collection '{self.collection_name}'.")

    def run_pipeline(self):
        """Run the full embedding pipeline."""
        logger.info("Starting Docusaurus to RAG embedding pipeline...")

        # Step 1: Extract text
        documents = self.crawl_and_extract(self.target_url)

        if not documents:
            logger.error("No documents extracted. Aborting.")
            return

        # Step 2: Chunk
        chunks = self.chunk_text(documents)

        # Step 3: Embed
        embedded_chunks = self.embed_chunks(chunks)

        # Step 4: Setup and upload to Qdrant
        self.create_collection()
        self.upload_to_qdrant(embedded_chunks)

        logger.info("RAG embedding pipeline completed successfully!")


# Example usage
if __name__ == "__main__":
    pipeline = DocusaurusEmbeddingPipeline()
    pipeline.run_pipeline()