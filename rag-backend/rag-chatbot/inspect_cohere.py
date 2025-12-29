import cohere
import inspect

print("Cohere Version:", cohere.__version__)
print("\nClient Init Signature:")
print(inspect.signature(cohere.Client.__init__))

print("\nClient Attributes:")
client = cohere.Client("dummy_key")
print(dir(client))
