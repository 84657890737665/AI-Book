"""Text Extractor service for the RAG Pipeline."""
import requests
from typing import List, Dict, Any, Optional
from bs4 import BeautifulSoup
from urllib.parse import urljoin
from ..models.book_content import BookContent
from ..lib.logging import logger
from ..lib.errors import TextExtractionError


class TextExtractor:
    """Service to extract clean text content from HTML pages."""
    
    def __init__(self, session: Optional[requests.Session] = None):
        """
        Initialize the Text Extractor.
        
        Args:
            session: Optional requests session to use (for connection pooling, etc.)
        """
        self.session = session or requests.Session()
        self.session.headers.update({
            'User-Agent': 'RAG-Pipeline/1.0 (compatible; bot)'
        })
    
    def extract_content(self, urls: List[str], content_selector: Optional[str] = None) -> List[BookContent]:
        """
        Extract clean text content from a list of URLs.
        
        Args:
            urls: List of URLs to extract content from
            content_selector: Optional CSS selector for content (e.g., 'article', '.content', '#main')
            
        Returns:
            List of BookContent objects containing the extracted content
        """
        extracted_content = []
        failed_extractions = []
        
        for url in urls:
            try:
                logger.info(f"Extracting content from: {url}")
                
                # Fetch the page content
                response = self.session.get(url, timeout=10)
                response.raise_for_status()
                
                # Parse the HTML
                soup = BeautifulSoup(response.text, 'html.parser')
                
                # Extract title
                title_tag = soup.find('title')
                title = title_tag.get_text().strip() if title_tag else f"Page from {url}"
                
                # Extract text content
                if content_selector:
                    # Use the provided selector to extract specific content
                    content_element = soup.select_one(content_selector)
                    raw_text = content_element.get_text().strip() if content_element else ""
                else:
                    # Remove script and style elements
                    for script in soup(["script", "style"]):
                        script.decompose()
                    
                    # Extract all text
                    raw_text = soup.get_text()
                
                # Clean up the text (remove extra whitespace)
                clean_text = self._clean_text(raw_text)
                
                # Create BookContent object
                content = BookContent(
                    id=f"content_{hash(url)}",
                    url=url,
                    title=title,
                    raw_text=clean_text,
                    clean_text=clean_text
                )
                
                extracted_content.append(content)
                
            except Exception as e:
                logger.error(f"Failed to extract content from {url}: {e}")
                failed_extractions.append(url)
        
        logger.info(f"Successfully extracted content from {len(extracted_content)} URLs")
        if failed_extractions:
            logger.warning(f"Failed to extract content from {len(failed_extractions)} URLs: {failed_extractions}")
        
        return extracted_content
    
    def _clean_text(self, text: str) -> str:
        """
        Clean up extracted text by removing extra whitespace and normalizing.
        
        Args:
            text: The raw text to clean
            
        Returns:
            Cleaned text
        """
        if not text:
            return ""
        
        # Split text into lines and remove empty lines
        lines = text.splitlines()
        non_empty_lines = [line.strip() for line in lines if line.strip()]
        
        # Join lines with single newlines
        clean_text = '\n'.join(non_empty_lines)
        
        # Normalize multiple spaces to single space
        import re
        clean_text = re.sub(r'\s+', ' ', clean_text)
        
        return clean_text.strip()
    
    def deduplicate_content(self, content_list: List[BookContent]) -> List[BookContent]:
        """
        Remove duplicate content from a list of BookContent objects.
        
        Args:
            content_list: List of BookContent objects
            
        Returns:
            List of BookContent objects with duplicates removed
        """
        seen_content = set()
        unique_content = []
        
        for content in content_list:
            # Use a combination of URL and content hash to identify duplicates
            content_key = (content.url, hash(content.raw_text))
            
            if content_key not in seen_content:
                seen_content.add(content_key)
                unique_content.append(content)
            else:
                logger.info(f"Removed duplicate content from URL: {content.url}")
        
        logger.info(f"Deduplication removed {len(content_list) - len(unique_content)} duplicate entries")
        return unique_content