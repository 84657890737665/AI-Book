"""Utility functions for the RAG Pipeline."""
import json
import pickle
from pathlib import Path
from typing import Any, Dict, List, Optional


def save_json(data: Any, file_path: str) -> None:
    """
    Save data to a JSON file.
    
    Args:
        data: The data to save
        file_path: Path to the file to save to
    """
    path = Path(file_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def load_json(file_path: str) -> Any:
    """
    Load data from a JSON file.
    
    Args:
        file_path: Path to the file to load from
        
    Returns:
        The loaded data
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        return json.load(f)


def save_pickle(data: Any, file_path: str) -> None:
    """
    Save data to a pickle file.
    
    Args:
        data: The data to save
        file_path: Path to the file to save to
    """
    path = Path(file_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(path, 'wb') as f:
        pickle.dump(data, f)


def load_pickle(file_path: str) -> Any:
    """
    Load data from a pickle file.
    
    Args:
        file_path: Path to the file to load from
        
    Returns:
        The loaded data
    """
    with open(file_path, 'rb') as f:
        return pickle.load(f)


def ensure_directory_exists(path: str) -> Path:
    """
    Ensure that a directory exists, creating it if necessary.
    
    Args:
        path: Path to the directory
        
    Returns:
        Path object for the directory
    """
    dir_path = Path(path)
    dir_path.mkdir(parents=True, exist_ok=True)
    return dir_path


def read_file(file_path: str) -> str:
    """
    Read the contents of a file.
    
    Args:
        file_path: Path to the file to read
        
    Returns:
        The file contents as a string
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        return f.read()


def write_file(file_path: str, content: str) -> None:
    """
    Write content to a file.
    
    Args:
        file_path: Path to the file to write to
        content: The content to write
    """
    path = Path(file_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(path, 'w', encoding='utf-8') as f:
        f.write(content)