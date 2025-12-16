import asyncio
import cohere
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.conversions import common_types
from config.settings import settings
from src.models.processed_content import ProcessedContent


class EmbeddingService:
    """
    Service to handle embedding generation and storage in Qdrant vector database.
    """
    
    def __init__(self):
        # Initialize Cohere client
        self.cohere_client = cohere.Client(settings.cohere_api_key)
        
        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=settings.qdrant_host,
            api_key=settings.qdrant_api_key,
            # timeout=10
        )
        
        # Define vector size for Cohere embeddings (multilingual-v3.0 returns 1024 dimensions)
        self.vector_size = 1024
        
    def create_collection(self, collection_name: str) -> bool:
        """
        Create a collection in Qdrant for storing embeddings.
        """
        try:
            # Check if collection already exists
            collections_response = self.qdrant_client.get_collections()
            existing_collections = [collection.name for collection in collections_response.collections]
            
            if collection_name not in existing_collections:
                # Create the collection
                self.qdrant_client.create_collection(
                    collection_name=collection_name,
                    vectors_config=models.VectorParams(
                        size=self.vector_size,
                        distance=models.Distance.COSINE
                    )
                )
                print(f"Collection '{collection_name}' created successfully.")
            else:
                print(f"Collection '{collection_name}' already exists.")
                
            return True
        except Exception as e:
            print(f"Error creating collection '{collection_name}': {str(e)}")
            return False
    
    def generate_embeddings(self, texts: List[str]) -> Optional[List[List[float]]]:
        """
        Generate embeddings for a list of texts using Cohere.
        """
        try:
            response = self.cohere_client.embed(
                texts=texts,
                model=settings.cohere_model_embedding,
                input_type="search_document"  # Suitable for document search
            )
            
            # Extract embeddings from the response
            embeddings = [embedding for embedding in response.embeddings]
            return embeddings
        except Exception as e:
            print(f"Error generating embeddings: {str(e)}")
            return None
    
    def store_embeddings(self, collection_name: str, embeddings: List[List[float]], 
                         payloads: List[Dict[str, Any]], ids: List[str]) -> bool:
        """
        Store embeddings in Qdrant collection.
        """
        try:
            # Prepare points for insertion
            points = []
            for idx, (embedding, payload, point_id) in enumerate(zip(embeddings, payloads, ids)):
                points.append(models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload=payload
                ))
            
            # Upsert points to the collection
            self.qdrant_client.upsert(
                collection_name=collection_name,
                points=points
            )
            
            print(f"Stored {len(points)} embeddings in collection '{collection_name}'.")
            return True
        except Exception as e:
            print(f"Error storing embeddings in collection '{collection_name}': {str(e)}")
            return False
    
    def process_and_store_book_embeddings(self, book_id: str, processed_contents: List[ProcessedContent]) -> bool:
        """
        Process and store embeddings for a book's processed content.
        """
        try:
            # Create collection named after the book ID
            collection_name = f"book_{book_id.replace('-', '_')}"  # Replace hyphens with underscores for valid collection name
            if not self.create_collection(collection_name):
                return False
            
            # Prepare batch processing of texts
            batch_size = 96  # Cohere's max batch size is 96
            success_count = 0
            
            for i in range(0, len(processed_contents), batch_size):
                batch = processed_contents[i:i + batch_size]
                
                # Extract texts and metadata
                texts = [pc.chunk_text for pc in batch]
                payloads = [{
                    "book_id": str(pc.book_id),
                    "chunk_id": pc.chunk_id,
                    "chunk_metadata": pc.chunk_metadata
                } for pc in batch]
                ids = [pc.id for pc in batch]
                
                # Generate embeddings
                embeddings = self.generate_embeddings(texts)
                if embeddings is None:
                    print(f"Failed to generate embeddings for batch starting at index {i}")
                    continue
                
                # Update embedding IDs in the database
                db_batch = []
                for pc, embedding_id in zip(batch, ids):
                    pc.embedding_id = embedding_id
                    db_batch.append(pc)
                
                # Store embeddings in Qdrant
                if self.store_embeddings(collection_name, embeddings, payloads, ids):
                    success_count += len(batch)
                else:
                    print(f"Failed to store embeddings for batch starting at index {i}")
            
            print(f"Successfully processed and stored {success_count}/{len(processed_contents)} chunks.")
            return success_count == len(processed_contents)
            
        except Exception as e:
            print(f"Error processing and storing embeddings for book {book_id}: {str(e)}")
            return False
    
    def search_similar(self, collection_name: str, query_text: str, top_k: int = 5) -> Optional[List[Dict[str, Any]]]:
        """
        Search for similar chunks in the collection based on the query text.
        """
        try:
            # Generate embedding for the query
            query_embedding = self.generate_embeddings([query_text])
            if query_embedding is None:
                return None
            
            # Search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=collection_name,
                query_vector=query_embedding[0],
                limit=top_k
            )
            
            # Format results
            results = []
            for result in search_results:
                results.append({
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload,
                    "text": result.payload.get("text", "")
                })
            
            return results
        except Exception as e:
            print(f"Error searching for similar content in collection '{collection_name}': {str(e)}")
            return None