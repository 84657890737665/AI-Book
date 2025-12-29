from qdrant_client import QdrantClient
import inspect

print("QdrantClient Init Signature:")
print(inspect.signature(QdrantClient.__init__))
