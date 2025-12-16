import hashlib
import secrets
from typing import Optional
from datetime import datetime, timedelta
from jose import JWTError, jwt
from config.settings import settings


def hash_sensitive_data(data: str) -> str:
    """
    Hash sensitive data using SHA-256.
    
    Args:
        data: The sensitive data to hash
        
    Returns:
        A hexadecimal SHA-256 hash of the data
    """
    return hashlib.sha256(data.encode()).hexdigest()


def generate_secure_token(length: int = 32) -> str:
    """
    Generate a cryptographically secure random token.
    
    Args:
        length: The length of the token in bytes (default 32 bytes = 256 bits)
        
    Returns:
        A secure random token as a hexadecimal string
    """
    return secrets.token_hex(length)


def mask_sensitive_info(info: str, visible_chars: int = 3) -> str:
    """
    Mask sensitive information, showing only a few leading characters.
    
    Args:
        info: The sensitive information to mask
        visible_chars: Number of leading characters to show (default 3)
        
    Returns:
        Masked string with leading characters visible and the rest replaced with '*'
    """
    if len(info) <= visible_chars:
        return "*" * len(info)
    
    return info[:visible_chars] + "*" * (len(info) - visible_chars)


def verify_access_token(token: str) -> Optional[dict]:
    """
    Verify an access token and return the payload if valid.
    
    Args:
        token: The JWT token to verify
        
    Returns:
        The payload dictionary if token is valid, None otherwise
    """
    try:
        payload = jwt.decode(token, settings.secret_key, algorithms=[settings.algorithm])
        return payload
    except JWTError:
        return None


def is_token_expired(expires_at: datetime) -> bool:
    """
    Check if a token has expired.
    
    Args:
        expires_at: The expiration datetime of the token
        
    Returns:
        True if the token has expired, False otherwise
    """
    return datetime.utcnow() > expires_at


def get_time_until_expiry(expires_at: datetime) -> timedelta:
    """
    Calculate the time remaining until a token expires.
    
    Args:
        expires_at: The expiration datetime of the token
        
    Returns:
        A timedelta representing the time until expiry
        (negative if already expired)
    """
    return expires_at - datetime.utcnow()


# Additional security-related functions can be added here


def escape_html(text: str) -> str:
    """
    Escape HTML characters to prevent XSS attacks.
    
    Args:
        text: The text to escape HTML characters in
        
    Returns:
        The text with HTML characters escaped
    """
    html_escape_table = {
        "&": "&amp;",
        '"': "&quot;",
        "'": "&#39;",
        ">": "&gt;",
        "<": "&lt;",
    }
    return "".join(html_escape_table.get(char, char) for char in text)