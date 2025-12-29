"""Unit tests for the URL Fetcher service."""
import pytest
import requests
from unittest.mock import Mock, patch
from src.services.url_fetcher import URLFetcher
from src.lib.errors import URLFetchError


class TestURLFetcher:
    """Test cases for the URLFetcher class."""
    
    def test_initialization(self):
        """Test that URLFetcher initializes correctly."""
        fetcher = URLFetcher("https://example.com")
        assert fetcher.base_url == "https://example.com"
        assert fetcher.domain == "https://example.com"
    
    @patch('requests.Session.get')
    def test_validate_url_success(self, mock_get):
        """Test that validate_url returns True for successful requests."""
        mock_get.return_value.status_code = 200
        
        fetcher = URLFetcher("https://example.com")
        result = fetcher.validate_url("https://example.com/page")
        
        assert result is True
        mock_get.assert_called_once()
    
    @patch('requests.Session.get')
    def test_validate_url_failure(self, mock_get):
        """Test that validate_url returns False for failed requests."""
        mock_get.side_effect = requests.RequestException("Connection error")
        
        fetcher = URLFetcher("https://example.com")
        result = fetcher.validate_url("https://example.com/page")
        
        assert result is False
        assert mock_get.call_count == 2  # HEAD request fails, then GET request also fails