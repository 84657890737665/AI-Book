# Constants for the RAG Chatbot application

# File upload settings
MAX_UPLOAD_SIZE = 50 * 1024 * 1024  # 50 MB
ALLOWED_EXTENSIONS = {".pdf", ".epub", ".txt"}

# Processing settings
CHUNK_SIZE = 1000  # Characters per chunk
CHUNK_OVERLAP = 100  # Overlapping characters between chunks

# API settings
DEFAULT_TOP_K = 5  # Default number of retrieved results
MAX_QUERY_LENGTH = 2000  # Maximum length for query text
MAX_SELECTED_TEXT_LENGTH = 5000  # Maximum length for selected text

# Timeout settings
QUERY_TIMEOUT = 30  # Seconds for query timeout
EMBEDDING_TIMEOUT = 60  # Seconds for embedding generation timeout

# Cohere model specifications
COHERE_GENERATION_MODEL = "command-r-plus"
COHERE_EMBEDDING_MODEL = "embed-multilingual-v3.0"

# Response settings
GENERATION_TEMPERATURE = 0.3  # Temperature for generation
MAX_TOKENS = 500  # Maximum tokens in response