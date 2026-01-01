---
title: RAG Chatbot API
emoji: 🤖
colorFrom: blue
colorTo: indigo
sdk: docker
app_port: 7860
---

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
- Query endpoint for RAG-based question answering

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
OPENROUTER_API_KEY=your_openrouter_api_key_here
```

For Hugging Face deployment, add these variables to your Space's secrets instead of committing a `.env` file.

## Usage

### Running Locally

```bash
# Run the main server
python server.py

# Or run the RAG query API directly
python api.py
```

The API will be available at `http://localhost:7860` (server.py) or `http://localhost:8000` (api.py)

### API Endpoints

- `POST /process-url` - Start processing a URL (server.py)
- `GET /job-status/{job_id}` - Get status of a processing job (server.py)
- `POST /search` - Search the vector database for relevant content (server.py)
- `GET /health` - Health check endpoint (server.py)
- `POST /query` - Query the RAG system for answers (api.py)
- `GET /` - Health check for the query API (api.py)

### Query API Usage

The main RAG Chatbot query endpoint is available at `/query` and accepts the following payload:

```json
{
  "query": "Your question here",
  "mode": "FULL_BOOK",  // or "SELECTED_TEXT"
  "selected_text": "Optional text for context when mode is SELECTED_TEXT"
}
```

#### Example Query Request

```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key principles of Physical AI?",
    "mode": "FULL_BOOK",
    "selected_text": null
  }'
```

#### Example Response

```json
{
  "response": "The key principles of Physical AI include...",
  "status": "success",
  "grounded_in_context": true,
  "confidence_score": 0.85,
  "citation_references": ["https://example.com/source1"]
}
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
7. **RAG Agent**: Processes queries using retrieval-augmented generation

## Frontend Integration

This backend API is designed to integrate with a frontend built with Docusaurus, Next.js, and custom CSS. The API provides endpoints that the frontend can call to initiate processing and track job status.

The `/query` endpoint is specifically designed to work with the Docusaurus chatbot frontend, supporting both full-book queries and queries scoped to selected text.