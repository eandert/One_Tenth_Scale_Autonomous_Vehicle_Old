import cryptography

from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives.asymmetric import rsa

#Asymmetric Keygen, 1 pair assigned per car
    #TODO: Bool in place to ensure this does not happen more than once upon startup
    #TODO: Keypair persists within secure enclave
private_key = rsa.generate_private_key(
    public_exponent=65537,
    key_size=2048,
    backend=default_backend()
)
public_key = private_key.public_key()