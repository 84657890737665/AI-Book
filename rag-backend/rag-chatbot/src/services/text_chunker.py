"""Text Chunker service for the RAG Pipeline."""
from typing import List
from langchain_text_splitters import RecursiveCharacterTextSplitter
from ..models.text_chunk import TextChunk
from ..lib.logging import logger


class TextChunker:
    """Service to split large text content into optimally sized chunks for RAG."""
    
    def __init__(self, chunk_size: int = 1000, chunk_overlap: int = 200, separator: str = "\n\n"):
        """
        Initialize the Text Chunker.
        
        Args:
            chunk_size: Size of text chunks (default: 1000 characters)
            chunk_overlap: Overlap between chunks (default: 200 characters)
            separator: Separator to use for chunking (default: \n\n)
        """
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        self.separator = separator
        
        # Initialize the text splitter
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=chunk_size,
            chunk_overlap=chunk_overlap,
            length_function=len,
            is_separator_regex=False,
            separators=[
                separator,
                "\n",
                " ",
                ""
            ]
        )
    
    def chunk_content(self, content_list: List[dict]) -> List[TextChunk]:
        """
        Split large text content into optimally sized chunks.
        
        Args:
            content_list: List of content items, each with 'id', 'text', 'url', and 'title'
            
        Returns:
            List of TextChunk objects
        """
        chunks = []
        
        for i, item in enumerate(content_list):
            content_id = item.get("id", f"content_{i}")
            text = item.get("text", "")
            url = item.get("url", "")
            title = item.get("title", "")
            
            logger.info(f"Chunking content from {url} (title: {title})")
            
            # Split the text into chunks
            split_texts = self.text_splitter.split_text(text)
            
            # Create TextChunk objects
            for idx, chunk_text in enumerate(split_texts):
                chunk = TextChunk(
                    id=f"chunk_{content_id}_{idx}",
                    content_id=content_id,
                    text=chunk_text,
                    source_url=url,
                    page_title=title,
                    chunk_index=idx,
                    token_count=len(chunk_text.split())  # Simple token count
                )
                
                chunks.append(chunk)
        
        logger.info(f"Created {len(chunks)} chunks from {len(content_list)} content items")
        return chunks