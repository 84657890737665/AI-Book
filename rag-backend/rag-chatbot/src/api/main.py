"""FastAPI application for the RAG Pipeline."""
from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import asyncio
import uuid
from src.services.url_fetcher import URLFetcher
from src.services.text_extractor import TextExtractor
from src.services.text_chunker import TextChunker
from src.services.embedding_generator import EmbeddingGenerator
from src.services.vector_storage import VectorStorage
from src.lib.config import Config
from src.lib.logging import logger
from src.lib.state_manager import ProcessingState
from src.lib.errors import handle_error


# Initialize FastAPI app
app = FastAPI(
    title="RAG Pipeline API",
    description="API for ingesting website content, generating embeddings, and storing in vector database",
    version="0.1.0"
)


# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://ai-book-topaz.vercel.app/", "http://localhost:3000"],  # In production, specify your frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Add global vector storage instance for search functionality
# In a production environment, you'd want to manage this differently (e.g., dependency injection)
vector_storage_instance = None

@app.on_event("startup")
async def startup_event():
    """Initialize the application on startup."""
    global vector_storage_instance

    logger.info("RAG Pipeline API starting up")
    # Validate configuration
    try:
        Config.validate()
        Config.validate_api_key_format()
        logger.info("Configuration validated successfully")

        # Initialize vector storage instance
        vector_storage_instance = VectorStorage()
        logger.info("Vector storage initialized successfully")
    except ValueError as e:
        logger.error(f"Configuration validation failed: {e}")
        raise


# Request/Response models
class ProcessURLRequest(BaseModel):
    url: str
    chunk_size: Optional[int] = Config.CHUNK_SIZE
    chunk_overlap: Optional[int] = Config.CHUNK_OVERLAP
    batch_size: Optional[int] = Config.BATCH_SIZE
    resume: Optional[bool] = False
    force_reprocess: Optional[bool] = False


class ProcessURLResponse(BaseModel):
    job_id: str
    status: str
    message: str


class JobStatusResponse(BaseModel):
    job_id: str
    status: str
    progress: float
    details: Optional[Dict[str, Any]]


class SearchRequest(BaseModel):
    query: str
    top_k: Optional[int] = 5


class SearchResult(BaseModel):
    id: str
    score: float
    content: str
    source_url: str
    section_title: str


class SearchResponse(BaseModel):
    query: str
    results: List[SearchResult]


# In-memory job storage (in production, use a database)
jobs: Dict[str, Dict[str, Any]] = {}


@app.on_event("startup")
async def startup_event():
    """Initialize the application on startup."""
    logger.info("RAG Pipeline API starting up")
    # Validate configuration
    try:
        Config.validate()
        Config.validate_api_key_format()
        logger.info("Configuration validated successfully")
    except ValueError as e:
        logger.error(f"Configuration validation failed: {e}")
        raise


@app.get("/")
async def root():
    """Root endpoint to verify the API is running."""
    return {"message": "RAG Pipeline API is running", "version": "0.1.0"}


@app.post("/process-url", response_model=ProcessURLResponse)
async def process_url(request: ProcessURLRequest, background_tasks: BackgroundTasks):
    """Start processing a URL in the background."""
    job_id = str(uuid.uuid4())
    
    # Store job info
    jobs[job_id] = {
        "status": "queued",
        "progress": 0.0,
        "details": {
            "url": request.url,
            "chunk_size": request.chunk_size,
            "chunk_overlap": request.chunk_overlap,
            "batch_size": request.batch_size,
            "resume": request.resume,
            "force_reprocess": request.force_reprocess
        }
    }
    
    # Run the processing in the background
    background_tasks.add_task(run_pipeline, job_id, request)
    
    return ProcessURLResponse(
        job_id=job_id,
        status="queued",
        message=f"Processing started for {request.url}"
    )


@app.get("/job-status/{job_id}", response_model=JobStatusResponse)
async def get_job_status(job_id: str):
    """Get the status of a processing job."""
    if job_id not in jobs:
        raise HTTPException(status_code=404, detail="Job not found")
    
    return JobStatusResponse(
        job_id=job_id,
        status=jobs[job_id]["status"],
        progress=jobs[job_id]["progress"],
        details=jobs[job_id]["details"]
    )


@app.post("/search", response_model=SearchResponse)
async def search(request: SearchRequest):
    """Search the vector database for relevant content based on the query."""
    global vector_storage_instance

    if vector_storage_instance is None:
        raise HTTPException(status_code=500, detail="Vector storage not initialized")

    try:
        # Generate embedding for the query
        from src.services.embedding_generator import EmbeddingGenerator
        embedding_gen = EmbeddingGenerator()

        # Create a chunk for the query to generate embedding
        query_chunk = [{
            "id": "query_chunk",
            "text": request.query
        }]

        # Generate embedding for the query
        query_embeddings = embedding_gen.generate_embeddings(query_chunk)

        if not query_embeddings:
            raise HTTPException(status_code=400, detail="Could not generate embedding for query")

        # Get the query vector from the first embedding
        query_vector = query_embeddings[0].vector

        # Search in the vector database
        search_results = vector_storage_instance.search(query_vector, top_k=request.top_k)

        # Format results
        formatted_results = []
        for result in search_results:
            payload = result.get("payload", {})
            content = payload.get("section_title", "")
            formatted_results.append(
                SearchResult(
                    id=result["id"],
                    score=result["score"],
                    content=content[:200] + "..." if len(content) > 200 else content,
                    source_url=payload.get("source_url", ""),
                    section_title=payload.get("section_title", "")
                )
            )

        return SearchResponse(query=request.query, results=formatted_results)

    except Exception as e:
        logger.error(f"Search failed: {e}")
        raise HTTPException(status_code=500, detail=f"Search failed: {str(e)}")


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}


async def run_pipeline(job_id: str, request: ProcessURLRequest):
    """Run the RAG pipeline for a given URL."""
    try:
        # Update job status
        jobs[job_id]["status"] = "processing"
        jobs[job_id]["progress"] = 0.1
        
        # Create pipeline components
        url_fetcher = URLFetcher(request.url)
        text_extractor = TextExtractor()
        text_chunker = TextChunker(
            chunk_size=request.chunk_size,
            chunk_overlap=request.chunk_overlap
        )
        embedding_generator = EmbeddingGenerator()
        vector_storage = VectorStorage()
        
        # Create state manager for this job
        state_manager = ProcessingState(job_id)
        
        # Reset state if force_reprocess is specified
        if request.force_reprocess:
            logger.info(f"Job {job_id}: Force reprocessing - resetting pipeline state")
            state_manager.reset()
        
        # Update status to running
        state_manager.update_status("running")
        jobs[job_id]["progress"] = 0.2
        
        # Phase 1: URL fetching
        logger.info(f"Job {job_id}: Starting URL fetching phase")
        if request.resume and state_manager.has_remaining_urls():
            logger.info(f"Job {job_id}: Resuming from saved state")
            urls = state_manager.get_remaining_urls()
        else:
            # Fetch URLs from the website
            urls = url_fetcher.fetch_urls(include_subpages=True)
            # Set remaining URLs in state manager
            state_manager.set_remaining_urls(urls)
        
        # Process each URL
        processed_urls = state_manager.get_processed_urls()
        remaining_urls = [url for url in state_manager.get_remaining_urls() if url not in processed_urls]
        
        for i, url in enumerate(remaining_urls):
            try:
                # Validate URL before processing
                if url_fetcher.validate_url(url):
                    logger.info(f"Job {job_id}: Successfully processed URL: {url}")
                    state_manager.add_processed_url(url)
                else:
                    logger.warning(f"Job {job_id}: Failed to validate URL: {url}")
            except Exception as e:
                logger.error(f"Job {job_id}: Error processing URL {url}: {e}")
                continue
            
            # Update progress
            jobs[job_id]["progress"] = 0.2 + (0.2 * (i + 1) / len(remaining_urls))
        
        logger.info(f"Job {job_id}: Completed URL fetching for {len(state_manager.get_processed_urls())} URLs")
        jobs[job_id]["progress"] = 0.4
        
        # Phase 2: Text processing and chunking
        logger.info(f"Job {job_id}: Starting text processing phase")
        processed_urls = state_manager.get_processed_urls()
        
        # Extract content from URLs
        book_contents = text_extractor.extract_content(processed_urls)
        
        # Deduplicate content
        deduplicated_contents = text_extractor.deduplicate_content(book_contents)
        
        # Prepare content for chunking
        content_items = []
        for content in deduplicated_contents:
            content_items.append({
                "id": content.id,
                "url": content.url,
                "title": content.title,
                "text": content.clean_text
            })
        
        # Chunk the content
        text_chunks = text_chunker.chunk_content(content_items)
        logger.info(f"Job {job_id}: Completed text processing and chunking into {len(text_chunks)} chunks")
        jobs[job_id]["progress"] = 0.6
        
        # Phase 3: Embedding generation
        logger.info(f"Job {job_id}: Starting embedding generation phase")
        
        # Prepare chunks for embedding generation
        chunks_for_embedding = []
        for chunk in text_chunks:
            chunks_for_embedding.append({
                "id": chunk.id,
                "text": chunk.text
            })
        
        # Generate embeddings
        embeddings = embedding_generator.generate_embeddings(chunks_for_embedding)
        
        # Validate embedding dimensions
        if not embedding_generator.validate_embedding_dimensions(embeddings):
            raise Exception("Embedding dimension validation failed")
        
        logger.info(f"Job {job_id}: Completed embedding generation for {len(embeddings)} chunks")
        jobs[job_id]["progress"] = 0.8
        
        # Phase 4: Vector storage
        logger.info(f"Job {job_id}: Starting vector storage phase")

        # Create metadata for each embedding
        metadata_list = []
        for i, embedding in enumerate(embeddings):
            # Get the corresponding chunk to create metadata
            chunk = text_chunks[i] if i < len(text_chunks) else None
            source_url = chunk.content_id if chunk else "unknown"

            from src.models.metadata import VectorMetadata
            metadata = VectorMetadata(
                id=f"meta_{embedding.id}",
                vector_id=embedding.id,
                source_url=source_url,
                section_title=chunk.text[:50] + "..." if chunk and len(chunk.text) > 50 else (chunk.text if chunk else "unknown"),
                chunk_identifier=embedding.chunk_id,
            )
            metadata_list.append(metadata)

        # Store embeddings in Qdrant
        stored_results = vector_storage.store_embeddings(embeddings, metadata_list)
        
        logger.info(f"Job {job_id}: Completed vector storage for {len(stored_results)} vectors")
        jobs[job_id]["progress"] = 0.95
        
        # Update status to completed
        state_manager.update_status("completed")
        jobs[job_id]["status"] = "completed"
        jobs[job_id]["progress"] = 1.0
        
        logger.info(f"Job {job_id}: Pipeline completed successfully")
        
    except Exception as e:
        handle_error(e, f"pipeline execution for job {job_id}", logger)
        jobs[job_id]["status"] = "failed"
        jobs[job_id]["progress"] = 1.0
        logger.error(f"Job {job_id}: Pipeline failed with error: {e}")