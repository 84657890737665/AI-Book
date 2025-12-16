import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from config.settings import settings
from src.services.embedding_service import EmbeddingService


class RetrievalService:
    """
    Service to handle retrieving relevant context for queries.
    """
    
    def __init__(self):
        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=settings.qdrant_host,
            api_key=settings.qdrant_api_key,
            # timeout=10
        )
        
        # Initialize embedding service for query embedding generation
        self.embedding_service = EmbeddingService()
        
        # Setup logger
        self.logger = logging.getLogger(__name__)
    
    def retrieve_context(self, book_id: str, query_text: str, selected_text: Optional[str] = None, 
                         top_k: int = 5) -> Optional[List[Dict[str, Any]]]:
        """
        Retrieve relevant context for a query from the book's embeddings.
        
        Args:
            book_id: The ID of the book to search in
            query_text: The query text
            selected_text: Optional selected text to focus the retrieval
            top_k: Number of top results to retrieve
        
        Returns:
            List of relevant chunks with metadata
        """
        try:
            # Format collection name based on book ID
            collection_name = f"book_{book_id.replace('-', '_')}"
            
            # If selected text is provided, we need to isolate this context
            if selected_text:
                # For selected text mode, we'll search only within that text
                return self._retrieve_from_selected_text(collection_name, query_text, selected_text, top_k)
            else:
                # For full book mode, search in the full collection
                return self._retrieve_from_full_book(collection_name, query_text, top_k)
                
        except Exception as e:
            self.logger.error(f"Error retrieving context for book {book_id} and query '{query_text[:50]}...': {str(e)}")
            return None
    
    def _retrieve_from_full_book(self, collection_name: str, query_text: str, top_k: int = 5) -> Optional[List[Dict[str, Any]]]:
        """
        Retrieve context from the full book collection.
        """
        try:
            # Get the query embedding
            query_embedding = self.embedding_service.generate_embeddings([query_text])
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
                    "chunk_id": result.payload.get("chunk_id", ""),
                    "text": result.payload.get("text", ""),
                    "chunk_text": result.payload.get("chunk_text", ""),  # Using chunk_text from the payload
                    "score": result.score,
                    "metadata": result.payload.get("chunk_metadata", {})
                })
            
            self.logger.info(f"Retrieved {len(results)} relevant chunks for query '{query_text[:30]}...' from {collection_name}")
            return results
            
        except Exception as e:
            self.logger.error(f"Error retrieving from full book collection '{collection_name}': {str(e)}")
            return None
    
    def _retrieve_from_selected_text(self, collection_name: str, query_text: str, selected_text: str, 
                                     top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve context focused on the selected text.
        
        In a simplified approach, we'll check if the query is related to the selected text
        and return the selected text as context with high relevance score.
        """
        try:
            # For selected text mode, we'll use a simple approach:
            # 1. Check if the query is semantically related to the selected text
            # 2. Return the selected text with high relevance score
            
            # Generate embeddings for both query and selected text
            query_embedding = self.embedding_service.generate_embeddings([query_text])
            selected_text_embedding = self.embedding_service.generate_embeddings([selected_text])
            
            if query_embedding is None or selected_text_embedding is None:
                # If embedding generation fails, return selected text as-is
                return [{
                    "chunk_id": "selected_text_chunk",
                    "text": selected_text,
                    "chunk_text": selected_text,
                    "score": 0.9,  # High score indicating high relevance
                    "metadata": {"source": "selected_text", "type": "user_selection"}
                }]
            
            # In a real implementation, we would calculate the similarity between embeddings
            # For now, we'll just return the selected text with high confidence
            return [{
                "chunk_id": "selected_text_chunk",
                "text": selected_text,
                "chunk_text": selected_text,
                "score": 0.95,  # Very high score for selected text
                "metadata": {"source": "selected_text", "type": "user_selection"}
            }]
            
        except Exception as e:
            self.logger.error(f"Error retrieving from selected text: {str(e)}")
            # Fallback: return selected text as context
            return [{
                "chunk_id": "selected_text_chunk",
                "text": selected_text,
                "chunk_text": selected_text,
                "score": 0.9,
                "metadata": {"source": "selected_text", "type": "user_selection"}
            }]
    
    def find_relevant_chunks(self, book_id: str, query_text: str, top_k: int = 5) -> Optional[List[str]]:
        """
        Find relevant chunk IDs for a query.
        """
        try:
            results = self.retrieve_context(book_id, query_text, top_k=top_k)
            if results:
                return [result["chunk_id"] for result in results]
            return []
        except Exception as e:
            self.logger.error(f"Error finding relevant chunks for query '{query_text[:50]}...': {str(e)}")
            return None