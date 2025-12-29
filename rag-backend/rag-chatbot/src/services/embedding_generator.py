"""Embedding Generator service for the RAG Pipeline."""
import time
import uuid
from typing import List, Dict, Any
from cohere import Client
from ..models.embedding_vector import EmbeddingVector
from ..lib.config import Config
from ..lib.logging import logger
from ..lib.errors import EmbeddingGenerationError


class EmbeddingGenerator:
    """Service to generate embeddings from text chunks using Cohere models."""
    
    def __init__(self, api_key: str = None, model_name: str = None):
        """
        Initialize the Embedding Generator.
        
        Args:
            api_key: Cohere API key (if not provided, will use from config)
            model_name: Name of the Cohere model to use (if not provided, will use from config)
        """
        self.api_key = api_key or Config.COHERE_API_KEY
        self.model_name = model_name or Config.COHERE_MODEL
        
        if not self.api_key:
            raise ValueError("Cohere API key is required")
        
        self.client = Client(api_key=self.api_key)
    
    def generate_embeddings(self, chunks: List[Dict[str, Any]]) -> List[EmbeddingVector]:
        """
        Generate embeddings from text chunks using Cohere models.
        
        Args:
            chunks: List of chunks with 'id' and 'text' keys
            
        Returns:
            List of EmbeddingVector objects
        """
        if not chunks:
            logger.warning("No chunks provided for embedding generation")
            return []
        
        # Extract text from chunks for batch processing
        texts = [chunk["text"] for chunk in chunks if "text" in chunk]
        
        if not texts:
            logger.warning("No valid text found in chunks for embedding generation")
            return []
        
        embeddings = []
        failed_embeddings = []
        
        # Process in batches to respect API limits
        batch_size = Config.BATCH_SIZE
        for i in range(0, len(texts), batch_size):
            batch_texts = texts[i:i + batch_size]
            batch_chunk_ids = [chunks[j]["id"] for j in range(i, min(i + batch_size, len(chunks)))]
            
            try:
                logger.info(f"Generating embeddings for batch {i//batch_size + 1} of {len(texts)//batch_size + 1}")
                
                # Retry logic for rate limits
                max_retries = 5
                response = None
                
                for attempt in range(max_retries):
                    try:
                        # Generate embeddings using Cohere
                        response = self.client.embed(
                            texts=batch_texts,
                            model=self.model_name,
                            input_type="search_document"  # Using search_document as default input type
                        )
                        break  # Success, exit retry loop
                    except Exception as e:
                        error_msg = str(e)
                        if "429" in error_msg or "Too Many Requests" in error_msg or "rate limit" in error_msg.lower():
                            if attempt == max_retries - 1:
                                raise  # Max retries reached, re-raise exception
                            
                            wait_time = 2 * (2 ** attempt)  # Exponential backoff: 2, 4, 8, 16, 32
                            logger.warning(f"Rate limit hit. Retrying in {wait_time}s (Attempt {attempt + 1}/{max_retries})")
                            time.sleep(wait_time)
                        else:
                            raise  # Not a rate limit error, re-raise immediately

                if not response:
                    raise EmbeddingGenerationError("Failed to get response after retries")
                
                # Create EmbeddingVector objects
                for idx, embedding_vector in enumerate(response.embeddings):
                    chunk_id = batch_chunk_ids[idx]
                    vector = embedding_vector
                    
                    # Get dimensions from the first vector (they should all be the same)
                    if idx == 0 and i == 0:
                        dimensions = len(vector)
                    
                    # Generate deterministic UUID based on chunk_id for Qdrant compatibility
                    # Qdrant requires IDs to be UUIDs or integers
                    point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, str(chunk_id)))
                    
                    embedding = EmbeddingVector(
                        id=point_id,
                        chunk_id=chunk_id,
                        vector=vector,
                        dimensions=dimensions,
                        model_used=self.model_name
                    )
                    
                    embeddings.append(embedding)
                
                # Add a small delay to respect rate limits
                time.sleep(1.0)  # Increased default delay to be safer
                
            except Exception as e:
                logger.error(f"Error generating embeddings for batch {i//batch_size + 1}: {e}")
                failed_embeddings.extend(batch_chunk_ids)
        
        logger.info(f"Successfully generated {len(embeddings)} embeddings, {len(failed_embeddings)} failed")
        
        if failed_embeddings:
            logger.warning(f"Failed to generate embeddings for chunks: {failed_embeddings}")
            raise EmbeddingGenerationError(f"Failed to generate embeddings for {len(failed_embeddings)} chunks")
        
        return embeddings
    
    def validate_embedding_dimensions(self, embeddings: List[EmbeddingVector]) -> bool:
        """
        Validate that all embeddings have consistent dimensions.
        
        Args:
            embeddings: List of EmbeddingVector objects
            
        Returns:
            True if all embeddings have consistent dimensions, False otherwise
        """
        if not embeddings:
            return True
        
        first_dimension = embeddings[0].dimensions
        for emb in embeddings:
            if emb.dimensions != first_dimension:
                logger.error(f"Embedding dimension mismatch: expected {first_dimension}, got {emb.dimensions}")
                return False
        
        logger.info(f"All embeddings have consistent dimensions of {first_dimension}")
        return True