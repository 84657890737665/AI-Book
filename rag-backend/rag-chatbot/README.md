# RAG Chatbot API

A FastAPI-based RAG (Retrieval-Augmented Generation) Pipeline for ingesting website content, generating embeddings, and storing in vector database for semantic search.

## Features

- URL ingestion and content extraction
- Text chunking for optimal RAG processing
- Embedding generation using Cohere models
- Vector storage in Qdrant database
- Background job processing
- Progress tracking
- RESTful API endpoints

## Installation

```bash
# Clone the repository
git clone <repository-url>
cd rag-chatbot

# Create a virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

## Setup

Create a `.env` file in the project root with the following variables (use `.env.example` as a template):

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
```

For Hugging Face deployment, add these variables to your Space's secrets instead of committing a `.env` file.

## Usage

### Running Locally

```bash
# Run the server
python server.py
```

The API will be available at `http://localhost:8000`

### API Endpoints

- `POST /process-url` - Start processing a URL
- `GET /job-status/{job_id}` - Get status of a processing job
- `POST /search` - Search the vector database for relevant content
- `GET /health` - Health check endpoint

### Example Usage

```bash
# Start processing a URL
curl -X POST "http://localhost:8000/process-url" \
  -H "Content-Type: application/json" \
  -d '{
    "url": "https://example.com",
    "chunk_size": 1000,
    "chunk_overlap": 200
  }'

# Check job status
curl "http://localhost:8000/job-status/{job_id}"

# Search for content in the vector database
curl -X POST "http://localhost:8000/search" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "your search query here",
    "top_k": 5
  }'
```

## Hugging Face Deployment

This application is designed to be deployed on Hugging Face Spaces using Docker.

## Architecture

The RAG Pipeline consists of:

1. **URL Fetcher**: Crawls and extracts content from websites
2. **Text Extractor**: Processes and cleans text content
3. **Text Chunker**: Splits content into optimally sized chunks
4. **Embedding Generator**: Creates vector embeddings using Cohere
5. **Vector Storage**: Stores embeddings in Qdrant vector database
6. **API Layer**: FastAPI endpoints for interaction

## Frontend Integration

This backend API is designed to integrate with a frontend built with Docusaurus, Next.js, and custom CSS. The API provides endpoints that the frontend can call to initiate processing and track job status.