"""Quick script to inspect what's actually stored in Qdrant"""
import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    prefer_grpc=False
)

# Get collection info
collection_info = client.get_collection("rag_pipeline")
print(f"Collection has {collection_info.points_count} points")

# Scroll through first 5 points to see their payloads
points = client.scroll(
    collection_name="rag_pipeline",
    limit=5,
    with_payload=True,
    with_vectors=False
)

print("\nFirst 5 points in collection:")
for i, point in enumerate(points[0], 1):
    print(f"\n{i}. Point ID: {point.id}")
    print(f"   Payload: {point.payload}")
