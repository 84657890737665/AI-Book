"""End-to-end test for the RAG Pipeline."""
import pytest
import os
from unittest.mock import Mock, patch
from src.cli.main import _run_url_fetching, _run_text_processing, _run_embedding_generation
from src.services.url_fetcher import URLFetcher
from src.services.text_extractor import TextExtractor
from src.services.text_chunker import TextChunker
from src.services.embedding_generator import EmbeddingGenerator
from src.lib.state_manager import ProcessingState


def test_end_to_end_pipeline():
    """Test the complete pipeline flow from URL fetching to text processing."""
    # Create a temporary output directory for this test
    import tempfile
    import shutil
    test_output_dir = tempfile.mkdtemp()
    
    try:
        # Create a mock URL fetcher
        fetcher = URLFetcher("https://example.com")
        
        # Create a state manager for the test
        state_manager = ProcessingState("test_pipeline", f"{test_output_dir}/test_state.json")
        
        # Mock the URL fetching to return some test URLs
        with patch.object(fetcher, 'fetch_urls') as mock_fetch_urls:
            mock_fetch_urls.return_value = ["https://example.com/page1", "https://example.com/page2"]
            
            with patch.object(fetcher, 'validate_url') as mock_validate_url:
                mock_validate_url.return_value = True
                
                # Run URL fetching
                urls = _run_url_fetching(fetcher, state_manager, Mock(url="https://example.com", output_dir=test_output_dir))
                
                # Verify URLs were fetched
                assert len(urls) == 2
                assert "https://example.com/page1" in urls
                assert "https://example.com/page2" in urls
        
        # Update state to include the processed URLs
        state_manager.set_remaining_urls([])
        for url in urls:
            state_manager.add_processed_url(url)
        
        # Create text extractor and chunker for the next step
        extractor = TextExtractor()
        chunker = TextChunker(chunk_size=200, chunk_overlap=50)
        
        # Mock the content extraction
        with patch.object(extractor.session, 'get') as mock_session_get:
            mock_response = Mock()
            mock_response.text = "<html><head><title>Test</title></head><body><p>This is test content for the RAG pipeline.</p></body></html>"
            mock_response.raise_for_status.return_value = None
            mock_session_get.return_value = mock_response
            
            # Run text processing
            text_chunks = _run_text_processing(extractor, chunker, state_manager, Mock(output_dir=test_output_dir))
            
            # Verify we got some chunks
            assert len(text_chunks) > 0
            
        print("End-to-end test completed successfully")
        
    finally:
        # Clean up temporary directory
        shutil.rmtree(test_output_dir)