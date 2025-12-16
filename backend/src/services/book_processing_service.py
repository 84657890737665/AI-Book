import os
import tempfile
from typing import List, Dict, Any, Tuple
from PyPDF2 import PdfReader
from ebooklib import epub
from bs4 import BeautifulSoup
import re
from uuid import uuid4
from datetime import datetime
from langchain_text_splitters import RecursiveCharacterTextSplitter
from config.database import SessionLocal
from config.settings import settings
from src.models.book_content import BookContent, ProcessingStatus, BookContentType
from src.models.processed_content import ProcessedContent


class BookProcessingService:
    """
    Service to process book content (PDF, EPUB, text) and chunk it for embedding.
    """
    
    def __init__(self):
        # Initialize text splitter
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=1000,  # Number of characters in each chunk
            chunk_overlap=100,  # Number of overlapping characters between chunks
            length_function=len,
        )

    def process_book_content(self, book_id: str, file_path: str) -> bool:
        """
        Main entry point to process a book file based on its type.
        """
        db = SessionLocal()
        try:
            # Update processing status to PROCESSING
            book = db.query(BookContent).filter(BookContent.id == book_id).first()
            if not book:
                return False
            
            book.processing_status = ProcessingStatus.PROCESSING
            book.updated_at = datetime.utcnow()
            db.commit()
            
            # Process based on content type
            success = False
            if book.content_type == BookContentType.PDF:
                success = self._process_pdf(file_path, book_id)
            elif book.content_type == BookContentType.EPUB:
                success = self._process_epub(file_path, book_id)
            elif book.content_type == BookContentType.TEXT:
                success = self._process_text(file_path, book_id)
            else:
                # Handle other formats or raise an exception
                book.processing_status = ProcessingStatus.FAILED
                db.commit()
                return False
            
            if success:
                book.processing_status = ProcessingStatus.COMPLETED
            else:
                book.processing_status = ProcessingStatus.FAILED
                
            book.updated_at = datetime.utcnow()
            db.commit()
            return success
            
        except Exception as e:
            print(f"Error processing book {book_id}: {str(e)}")
            book.processing_status = ProcessingStatus.FAILED
            book.updated_at = datetime.utcnow()
            db.commit()
            return False
        finally:
            db.close()

    def _process_pdf(self, file_path: str, book_id: str) -> bool:
        """
        Process a PDF file and convert it to chunks.
        """
        try:
            pdf_reader = PdfReader(file_path)
            text_content = ""
            
            for page_num, page in enumerate(pdf_reader.pages):
                text_content += f"\n--- PAGE {page_num + 1} ---\n"
                text_content += page.extract_text()
                
            return self._chunk_and_store(text_content, book_id)
        except Exception as e:
            print(f"Error processing PDF {file_path}: {str(e)}")
            return False

    def _process_epub(self, file_path: str, book_id: str) -> bool:
        """
        Process an EPUB file and convert it to chunks.
        """
        try:
            book = epub.read_epub(file_path)
            text_content = ""
            
            for item_id, item in book.get_items():
                if item.get_type() == epub.ITEM_DOCUMENT:
                    soup = BeautifulSoup(item.get_body_content(), 'html.parser')
                    
                    # Add section break
                    text_content += f"\n--- SECTION {item_id} ---\n"
                    text_content += soup.get_text()
                    
            return self._chunk_and_store(text_content, book_id)
        except Exception as e:
            print(f"Error processing EPUB {file_path}: {str(e)}")
            return False

    def _process_text(self, file_path: str, book_id: str) -> bool:
        """
        Process a plain text file and convert it to chunks.
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                text_content = file.read()
                
            return self._chunk_and_store(text_content, book_id)
        except Exception as e:
            print(f"Error processing text file {file_path}: {str(e)}")
            return False

    def _chunk_and_store(self, text_content: str, book_id: str) -> bool:
        """
        Split text content into chunks and store in the database.
        """
        try:
            # Clean up the text content
            text_content = re.sub(r'\n+', '\n', text_content)  # Replace multiple newlines with single
            text_content = text_content.strip()
            
            # Split the text into chunks
            chunks = self.text_splitter.split_text(text_content)
            
            db = SessionLocal()
            try:
                # Store each chunk in the database
                for i, chunk in enumerate(chunks):
                    chunk_metadata = {
                        "chunk_index": i,
                        "total_chunks": len(chunks),
                        "source_type": "book_content"
                    }
                    
                    processed_chunk = ProcessedContent(
                        book_id=book_id,
                        chunk_id=str(uuid4()),
                        chunk_text=chunk,
                        chunk_metadata=chunk_metadata,
                        embedding_id=""  # Will be filled when embeddings are generated
                    )
                    
                    db.add(processed_chunk)
                
                db.commit()
                return True
            finally:
                db.close()
        except Exception as e:
            print(f"Error chunking and storing content for book {book_id}: {str(e)}")
            return False

    def preprocess_multimedia_content(self, multimedia_items: List[Dict[str, Any]]) -> List[Dict[str, str]]:
        """
        Preprocess multimedia content to extract text descriptions.
        """
        processed_items = []
        
        for item in multimedia_items:
            item_info = {
                "id": str(uuid4()),
                "content_type": item.get("content_type", ""),
                "description": item.get("description", ""),
                "alt_text": item.get("alt_text", ""),
                "source_path": item.get("source_path", "")
            }
            processed_items.append(item_info)
        
        return processed_items