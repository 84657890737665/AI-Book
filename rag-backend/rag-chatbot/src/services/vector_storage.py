"""Vector Storage service for the RAG Pipeline."""
from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from ..models.embedding_vector import EmbeddingVector
from ..models.metadata import VectorMetadata
from ..lib.config import Config
from ..lib.logging import logger
from ..lib.errors import VectorStorageError


class VectorStorage:
    """Service to store embeddings and metadata in Qdrant vector database."""
    
    def __init__(self, api_key: str = None, url: str = None, collection_name: str = "rag_pipeline"):
        """
        Initialize the Vector Storage.
        
        Args:
            api_key: Qdrant API key (if not provided, will use from config)
            url: Qdrant URL (if not provided, will use from config)
            collection_name: Name of the collection to store vectors in
        """
        self.api_key = api_key or Config.QDRANT_API_KEY
        self.url = url or Config.QDRANT_URL
        self.collection_name = collection_name
        
        if not self.api_key or not self.url:
            raise ValueError("Both Qdrant API key and URL are required")
        
        # Initialize Qdrant client
        self.client = QdrantClient(
            url=self.url,
            api_key=self.api_key,
            prefer_grpc=False,  # Using HTTP for better compatibility
            timeout=300  # Increased timeout to 300 for maximum stability
        )
        
        # Create collection if it doesn't exist
        self._create_collection_if_not_exists()
    
    def _create_collection_if_not_exists(self):
        """Create the collection if it doesn't already exist."""
        try:
            # Try to get collection info to check if it exists
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection '{self.collection_name}' already exists")
        except Exception:
            # Collection doesn't exist, create it
            logger.info(f"Creating collection '{self.collection_name}'")
            
            # Get a sample embedding to determine vector size
            # In a real scenario, this would come from your embedding model
            # For now, we'll use a default size of 768 (common for many models)
            # But in practice, you'd want to determine this dynamically
            vector_size = 768  # This should be determined based on your embedding model
            
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
            )
            
            logger.info(f"Collection '{self.collection_name}' created successfully")
    
    def store_embeddings(self, embeddings: List[EmbeddingVector], metadata_list: List[VectorMetadata] = None) -> List[Dict[str, Any]]:
        """
        Store embeddings and metadata in Qdrant vector database.

        Args:
            embeddings: List of EmbeddingVector objects to store
            metadata_list: Optional list of VectorMetadata objects to store with embeddings

        Returns:
            List of dictionaries containing chunk_id and vector_id for each stored vector
        """
        if not embeddings:
            logger.warning("No embeddings provided for storage")
            return []

        # Prepare points for Qdrant
        points = []
        for i, embedding in enumerate(embeddings):
            # Prepare payload (metadata)
            payload = {
                "chunk_id": embedding.chunk_id,
                "model_used": embedding.model_used,
                "created_at": embedding.created_at.isoformat(),
            }

            # Add additional metadata if provided
            if metadata_list and i < len(metadata_list):
                metadata = metadata_list[i]
                payload.update({
                    "source_url": metadata.source_url,
                    "section_title": metadata.section_title,
                    "chunk_identifier": metadata.chunk_identifier,
                    "chunk_text": metadata.chunk_text,  # Store the actual chunk content
                    "additional_metadata": metadata.additional_metadata
                })

            # Create point
            point = models.PointStruct(
                id=embedding.id,  # Using the embedding's ID as the point ID
                vector=embedding.vector,
                payload=payload
            )

            points.append(point)

        # Upload points to Qdrant in batches
        total_stored = 0
        batch_size = 10  # Reduced batch size further to 10 to avoid connection resets
        
        try:
            for i in range(0, len(points), batch_size):
                batch = points[i:i + batch_size]
                logger.info(f"Storing batch {i//batch_size + 1} ({len(batch)} vectors) in Qdrant collection '{self.collection_name}'")

                self.client.upsert(
                    collection_name=self.collection_name,
                    points=batch
                )
                total_stored += len(batch)


            # Prepare return value
            stored_vectors = []
            for embedding in embeddings:
                stored_vectors.append({
                    "chunk_id": embedding.chunk_id,
                    "vector_id": embedding.id,
                    "stored_at": embedding.created_at.isoformat()
                })

            logger.info(f"Successfully stored {len(stored_vectors)} vectors in Qdrant")
            return stored_vectors

        except Exception as e:
            logger.error(f"Error storing embeddings in Qdrant: {e}")
            raise VectorStorageError(f"Failed to store embeddings: {str(e)}") from e

    def get_collection_info(self):
        """
        Get information about the collection.

        Returns:
            Collection info including vector count
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                "name": collection_info.config.params.vectors.size,
                "vector_size": collection_info.config.params.vectors.size,
                "count": collection_info.points_count
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {e}")
            raise VectorStorageError(f"Failed to get collection info: {str(e)}") from e
    
    def search(self, query_vector: List[float], top_k: int = 10) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the collection.
        
        Args:
            query_vector: The vector to search for similar vectors to
            top_k: Number of top results to return
            
        Returns:
            List of dictionaries containing search results with payload and similarity scores
        """
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k
            )
            
            search_results = []
            for result in results:
                search_results.append({
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload,
                    "vector": result.vector
                })
            
            logger.info(f"Search returned {len(search_results)} results")
            return search_results
            
        except Exception as e:
            logger.error(f"Error searching in Qdrant: {e}")
            raise VectorStorageError(f"Failed to search vectors: {str(e)}") from e
    
    def delete_collection(self):
        """Delete the entire collection (use with caution)."""
        try:
            self.client.delete_collection(self.collection_name)
            logger.info(f"Collection '{self.collection_name}' deleted successfully")
        except Exception as e:
            logger.error(f"Error deleting collection: {e}")
            raise VectorStorageError(f"Failed to delete collection: {str(e)}") from e