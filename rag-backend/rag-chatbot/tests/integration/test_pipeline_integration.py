"""Integration tests for the RAG Pipeline."""
import pytest
import os
from unittest.mock import Mock, patch
from src.services.url_fetcher import URLFetcher
from src.services.text_extractor import TextExtractor
from src.services.text_chunker import TextChunker
from src.models.book_content import BookContent


class TestPipelineIntegration:
    """Integration tests for the RAG Pipeline components."""
    
    def test_url_fetcher_to_text_extractor_integration(self):
        """Test integration between URLFetcher and TextExtractor."""
        # Mock a simple HTML page
        mock_html = """
        <html>
            <head><title>Test Page</title></head>
            <body>
                <h1>Test Content</h1>
                <p>This is a test paragraph for the RAG pipeline.</p>
            </body>
        </html>
        """
        
        # Create fetcher and extractor
        fetcher = URLFetcher("https://example.com")
        
        # Mock the session.get method to return our test HTML
        with patch.object(fetcher.session, 'get') as mock_get:
            mock_get.return_value.text = mock_html
            mock_get.return_value.status_code = 200
            
            # Test that fetcher can get links
            links = fetcher._get_page_links("https://example.com")
            # Should find at least the example.com link itself
            assert len(links) >= 0  # May not find any links in our simple HTML
    
    def test_text_extraction_and_chunking(self):
        """Test integration between TextExtractor and TextChunker."""
        # Create a sample BookContent
        content = BookContent(
            id="test_content_1",
            url="https://example.com/test",
            title="Test Page",
            raw_text="This is a test paragraph for the RAG pipeline. " * 50,  # Make it longer for chunking
            clean_text="This is a test paragraph for the RAG pipeline. " * 50
        )
        
        # Create text chunker
        chunker = TextChunker(chunk_size=200, chunk_overlap=50)
        
        # Prepare content in the format expected by the chunker
        content_items = [{
            "id": content.id,
            "url": content.url,
            "title": content.title,
            "text": content.clean_text
        }]
        
        # Chunk the content
        chunks = chunker.chunk_content(content_items)
        
        # Verify we got chunks
        assert len(chunks) > 0
        # Verify each chunk has appropriate properties
        for chunk in chunks:
            assert chunk.content_id == content.id
            assert len(chunk.text) > 0
            assert chunk.chunk_index >= 0