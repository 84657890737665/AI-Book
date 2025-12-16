# Quickstart Guide: Integrated RAG Chatbot

**Feature**: Integrated RAG Chatbot Development
**Date**: 2025-12-13

## Overview

This guide provides a quick setup and usage guide for the Integrated RAG Chatbot. The system allows users to ask questions about book content using Retrieval-Augmented Generation (RAG) technology and to restrict queries to specific text selections within the book.

## Prerequisites

- Python 3.11+
- pip package manager
- Git
- Cohere API key (provided in project)
- Access to Neon Serverless Postgres (credentials provided)
- Access to Qdrant Cloud Free Tier (credentials provided)

## Setup

### 1. Clone the repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create a virtual environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install dependencies

```bash
pip install -r requirements.txt
```

### 4. Environment Configuration

Create a `.env` file in the project root with the following content:

```env
COHERE_API_KEY=ml0YZj88cPvNZbxW1ysKxMxbJTlcwOA5BcGeGjKO
NEON_DATABASE_URL=neondb_owner:npg_NeiVQ63cSDlg@ep-empty-cell-agjm0m46-pooler.c-2.eu-central-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require
QDRANT_HOST=https://2e580569-3a35-4d4c-bfc2-71189db2e78d.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.nLimUBljSFhYj8iztY5KZYJ5C5WgzAhi--kljHYJTOc
QDRANT_CLUSTER_ID=2e580569-3a35-4d4c-bfc2-71189db2e78d
```

### 5. Initialize the database

```bash
# Run database migrations
alembic upgrade head
```

### 6. Start the development server

```bash
uvicorn src.api.main:app --reload --port 8000
```

## Usage

### 1. Upload a book

To use the RAG chatbot, first upload a book in PDF or EPUB format:

```bash
curl -X POST "http://localhost:8000/books" \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -H "Content-Type: multipart/form-data" \
  -F "file=@path/to/your/book.pdf" \
  -F "title=Book Title" \
  -F "author=Author Name"
```

The book will be processed automatically, with chunks stored in Qdrant for retrieval.

### 2. Query the book content

Once processing is complete, you can ask questions about the book:

```bash
curl -X POST "http://localhost:8000/query/BOOK_ID" \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the main concepts covered in this book?",
    "mode": "full_book"
  }'
```

To query only specific selected text:

```bash
curl -X POST "http://localhost:8000/query/BOOK_ID" \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this concept in simpler terms",
    "mode": "selected_text",
    "selected_text": "The selected text that you want to focus on for this query..."
  }'
```

### 3. Embed in your book

To get the embeddable code for a book:

```bash
curl -X GET "http://localhost:8000/embed/BOOK_ID" \
  -H "Authorization: Bearer YOUR_JWT_TOKEN"
```

## API Endpoints

- `POST /auth/login` - Authenticate and get JWT token
- `POST /books` - Upload and process a new book
- `GET /books/{bookId}` - Get information about a book
- `POST /query/{bookId}` - Query book content using RAG
- `GET /embed/{bookId}` - Get embeddable chatbot code

## Security

- All endpoints except `/auth/login` require a valid JWT token
- Communication with Cohere API, Neon, and Qdrant is encrypted
- Sensitive credentials are stored in environment variables

## Troubleshooting

### Common Issues

1. **Database connection errors**: Verify your Neon credentials in the `.env` file
2. **Cohere API errors**: Confirm your API key is valid and has sufficient quota
3. **Qdrant connection issues**: Check the host URL and API key in your `.env` file
4. **PDF/EPUB processing failed**: Ensure the file format is supported and not corrupted

### Performance Tips

- Large books take longer to process initially
- Queries with selected text mode tend to be faster than full-book queries
- Responses include citation references to source passages

## Next Steps

1. Integrate the chatbot into your digital book format
2. Customize the embedded UI to match your book's style
3. Monitor usage and performance metrics
4. Explore advanced features like multi-book comparison