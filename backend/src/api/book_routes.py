from fastapi import APIRouter, UploadFile, File, Form, HTTPException, status, Depends
from typing import Optional
import uuid
import os
from datetime import datetime

from src.models.book_content import BookContent, ProcessingStatus, BookContentType
from src.services.book_processing_service import BookProcessingService
from src.services.embedding_service import EmbeddingService
from config.database import SessionLocal
from src.api.auth_routes import get_current_user

router = APIRouter()
book_processor = BookProcessingService()
embedding_service = EmbeddingService()


@router.post("/")
async def upload_book(
    file: UploadFile = File(...),
    title: str = Form(...),
    author: str = Form(...),
    current_user: str = Depends(get_current_user)
):
    """
    Upload and process a new book for the RAG system.
    """
    # Validate file type
    file_extension = file.filename.split(".")[-1].lower()
    if file_extension not in ["pdf", "epub", "txt"]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Unsupported file type: {file_extension}. Supported types: pdf, epub, txt"
        )
    
    # Create content type enum value
    if file_extension == "pdf":
        content_type = BookContentType.PDF
    elif file_extension == "epub":
        content_type = BookContentType.EPUB
    else:  # txt
        content_type = BookContentType.TEXT
        
    # Create book record in database
    db = SessionLocal()
    try:
        # Create unique filename
        unique_filename = f"{uuid.uuid4()}_{file.filename}"
        file_path = f"uploads/{unique_filename}"
        
        # Ensure upload directory exists
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        
        # Save the uploaded file
        with open(file_path, "wb") as f:
            f.write(await file.read())
        
        # Create book entry in the database
        book = BookContent(
            title=title,
            author=author,
            content_type=content_type,
            content_path=file_path,
            language="en",  # Default to English, could be detected
            processing_status=ProcessingStatus.PENDING
        )
        
        db.add(book)
        db.commit()
        db.refresh(book)
        
        # Start processing the book asynchronously (in a real implementation)
        # For now, we'll process synchronously
        success = book_processor.process_book_content(str(book.id), file_path)
        
        if success:
            # Generate embeddings for the processed content
            from src.models.processed_content import ProcessedContent
            processed_chunks = db.query(ProcessedContent).filter(ProcessedContent.book_id == book.id).all()
            
            if processed_chunks:
                embedding_success = embedding_service.process_and_store_book_embeddings(str(book.id), processed_chunks)
                if embedding_success:
                    book.processing_status = ProcessingStatus.COMPLETED
                else:
                    book.processing_status = ProcessingStatus.FAILED
            else:
                book.processing_status = ProcessingStatus.FAILED
        else:
            book.processing_status = ProcessingStatus.FAILED
            
        book.updated_at = datetime.utcnow()
        db.commit()
        db.refresh(book)
        
        return book
        
    except Exception as e:
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing book: {str(e)}"
        )
    finally:
        db.close()


@router.get("/{book_id}")
async def get_book_info(
    book_id: str,
    current_user: str = Depends(get_current_user)
):
    """
    Get information about a specific book.
    """
    db = SessionLocal()
    try:
        book = db.query(BookContent).filter(BookContent.id == book_id).first()
        
        if not book:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Book not found"
            )
        
        return book
    finally:
        db.close()