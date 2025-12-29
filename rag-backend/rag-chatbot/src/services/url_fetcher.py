"""URL Fetcher service for the RAG Pipeline."""
import requests
from typing import List, Dict, Any, Optional
from urllib.parse import urljoin, urlparse
from bs4 import BeautifulSoup
from ..lib.logging import logger
from ..lib.errors import URLFetchError


class URLFetcher:
    """Service to fetch and validate URLs from a given base URL."""
    
    def __init__(self, base_url: str, session: Optional[requests.Session] = None):
        """
        Initialize the URL Fetcher.
        
        Args:
            base_url: The base URL to start crawling from
            session: Optional requests session to use (for connection pooling, etc.)
        """
        self.base_url = base_url
        self.session = session or requests.Session()
        self.session.headers.update({
            'User-Agent': 'RAG-Pipeline/1.0 (compatible; bot)'
        })
        
        # Parse the base URL to extract domain
        parsed = urlparse(base_url)
        self.domain = f"{parsed.scheme}://{parsed.netloc}"
    
    def fetch_urls(self, include_subpages: bool = True, url_patterns: Optional[List[str]] = None) -> List[str]:
        """
        Fetch and validate URLs from the given base URL.
        
        Args:
            include_subpages: Whether to include subpages in the crawl
            url_patterns: Optional list of URL patterns to include
            
        Returns:
            List of valid URLs found on the site
        """
        logger.info(f"Starting URL fetch from: {self.base_url}")
        
        try:
            # Start with the base URL
            urls = [self.base_url]
            
            if include_subpages:
                # Get links from the base page
                base_page_links = self._get_page_links(self.base_url)
                urls.extend(base_page_links)
                
                # For each link, get its subpages as well (limited depth)
                for link in base_page_links[:10]:  # Limit to avoid infinite crawling
                    try:
                        subpage_links = self._get_page_links(link)
                        urls.extend(subpage_links)
                    except Exception as e:
                        logger.warning(f"Could not fetch subpages from {link}: {e}")
            
            # Filter URLs based on patterns if provided
            if url_patterns:
                filtered_urls = []
                for url in urls:
                    for pattern in url_patterns:
                        if pattern in url:
                            filtered_urls.append(url)
                            break
                urls = filtered_urls
            
            # Remove duplicates while preserving order
            unique_urls = list(dict.fromkeys(urls))
            
            logger.info(f"Found {len(unique_urls)} unique URLs")
            return unique_urls
            
        except Exception as e:
            logger.error(f"Error fetching URLs from {self.base_url}: {e}")
            raise URLFetchError(f"Failed to fetch URLs: {str(e)}") from e
    
    def _get_page_links(self, url: str) -> List[str]:
        """
        Get all links from a given page.
        
        Args:
            url: The URL to extract links from
            
        Returns:
            List of URLs found on the page
        """
        try:
            response = self.session.get(url, timeout=10)
            response.raise_for_status()
            
            soup = BeautifulSoup(response.text, 'html.parser')
            links = soup.find_all('a', href=True)
            
            urls = []
            for link in links:
                href = link['href']
                
                # Convert relative URLs to absolute URLs
                absolute_url = urljoin(url, href)
                
                # Only include URLs from the same domain
                if absolute_url.startswith(self.domain):
                    urls.append(absolute_url)
            
            return urls
            
        except requests.RequestException as e:
            logger.warning(f"Could not fetch page {url}: {e}")
            return []
        except Exception as e:
            logger.error(f"Error parsing page {url}: {e}")
            return []
    
    def validate_url(self, url: str) -> bool:
        """
        Validate if a URL is accessible and returns a successful response.
        
        Args:
            url: The URL to validate
            
        Returns:
            True if the URL is valid and accessible, False otherwise
        """
        try:
            response = self.session.head(url, timeout=5)
            return 200 <= response.status_code < 400
        except Exception:
            try:
                # If HEAD fails, try GET
                response = self.session.get(url, timeout=5)
                return 200 <= response.status_code < 400
            except Exception:
                return False