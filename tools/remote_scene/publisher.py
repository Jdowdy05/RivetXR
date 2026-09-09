"""Pinned-TLS-compatible, token-authenticated, latest-complete RSCN publisher."""
import argparse
import hashlib
import hmac
import ipaddress
import json
from pathlib import Path
import socket
import ssl
import struct
import threading
import time
from .protocol import MAX_MESSAGE,encode_transport,decode_transport,unpack_packet,SequenceHistory
from .recording import fields,load_json


def recv_exact(stream,count):
    result=bytearray()
    while len(result)<count:
        part=stream.recv(count-len(result))
        if not part:raise EOFError('remote stream ended')
        result.extend(part)
    return bytes(result)


def read_packet(stream):
    raw_length,compressed_length=struct.unpack('!II',recv_exact(stream,8))
    if not 128<=raw_length<=MAX_MESSAGE or not 0<compressed_length<=MAX_MESSAGE:raise ValueError('invalid transport frame lengths')
    return decode_transport(raw_length,recv_exact(stream,compressed_length))


def connect_pinned(config,timeout=2.,*,auth_magic=b'QRS1'):
    if auth_magic not in (b'QRS1',b'QGC1'):raise ValueError('invalid authenticated channel kind')
    fields(config,('version','host','port','certificate_sha256','token_hex'))
    if type(config['version']) is not int or config['version']!=1:raise ValueError('invalid client config version')
    if not isinstance(config['host'],str):raise ValueError('numeric host required')
    ipaddress.ip_address(config['host'])
    if type(config['port']) is not int or not 1<=config['port']<=65535:raise ValueError('invalid port')
    import re
    for key in ('certificate_sha256','token_hex'):
        if not isinstance(config[key],str) or not re.fullmatch('[0-9a-fA-F]{64}',config[key]):raise ValueError('invalid private pin/token config')
    context=ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT);context.minimum_version=ssl.TLSVersion.TLSv1_2
    context.check_hostname=False;context.verify_mode=ssl.CERT_NONE  # Explicit leaf pin replaces CA/hostname trust.
    raw=socket.create_connection((config['host'],config['port']),timeout=timeout)
    stream=None
    try:
        stream=context.wrap_socket(raw,server_hostname=None)
        if not hmac.compare_digest(hashlib.sha256(stream.getpeercert(binary_form=True)).hexdigest(),config['certificate_sha256'].lower()):
            stream.close();raise ValueError('server certificate pin mismatch')
        stream.sendall(auth_magic+bytes.fromhex(config['token_hex']))
        return stream
    except Exception:
        if stream is not None:stream.close()
        raw.close();raise


class LatestPublisher:
    """At most two clients and one latest wire message; no frame history queue."""
    def __init__(self,certificate,private_key,token,host='127.0.0.1',port=0):
        address=ipaddress.ip_address(host)
        if type(port) is not int or not 0<=port<=65535:raise ValueError('invalid port')
        if not isinstance(token,bytes) or len(token)!=32:raise ValueError('32-byte token required')
        self.context=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER);self.context.minimum_version=ssl.TLSVersion.TLSv1_2
        self.context.load_cert_chain(certificate,private_key)
        self.token=token;self.host=host;self.port=port;self.family=socket.AF_INET6 if address.version==6 else socket.AF_INET
        self.condition=threading.Condition();self.stopping=threading.Event();self.listener=None;self.accept_thread=None
        self.clients=set();self.threads=set();self.wire=None;self.version=0;self.identity=None;self.sequence=0
        self.history=SequenceHistory()
        self.rejected_clients=0;self.sent_frames=0

    @property
    def address(self):
        if self.listener is None:raise RuntimeError('publisher not started')
        return self.listener.getsockname()[:2]

    def start(self):
        if self.listener is not None or self.stopping.is_set():raise RuntimeError('publisher lifecycle cannot restart')
        listener=socket.socket(self.family,socket.SOCK_STREAM);listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        try:listener.bind((self.host,self.port));listener.listen(2);listener.settimeout(.2)
        except Exception:listener.close();raise
        self.listener=listener;self.accept_thread=threading.Thread(target=self._accept,name='rscn-accept',daemon=True);self.accept_thread.start()
        return self.address

    def publish(self,raw):
        packet=unpack_packet(raw);wire=encode_transport(raw)
        with self.condition:
            if self.stopping.is_set():raise RuntimeError('publisher stopped')
            self.history.accept(packet)
            self.identity=packet.identity;self.sequence=packet.sequence;self.wire=wire;self.version+=1;self.condition.notify_all()

    def _accept(self):
        while not self.stopping.is_set():
            try:raw,_=self.listener.accept()
            except socket.timeout:continue
            except OSError:break
            with self.condition:
                if self.stopping.is_set() or len(self.threads)>=2:raw.close();self.rejected_clients+=1;continue
                self.clients.add(raw)
                thread=threading.Thread(target=self._serve,args=(raw,),name='rscn-client',daemon=True)
                self.threads.add(thread);thread.start()

    def _serve(self,raw):
        stream=raw
        try:
            raw.settimeout(2.)
            stream=self.context.wrap_socket(raw,server_side=True,do_handshake_on_connect=False)
            with self.condition:self.clients.discard(raw);self.clients.add(stream)
            stream.do_handshake()
            request=recv_exact(stream,36)
            if request[:4]!=b'QRS1' or not hmac.compare_digest(request[4:],self.token):
                with self.condition:self.rejected_clients+=1
                return
            seen=0
            while not self.stopping.is_set():
                with self.condition:
                    self.condition.wait_for(lambda:self.stopping.is_set() or self.version!=seen,timeout=.2)
                    if self.stopping.is_set():break
                    if self.wire is None or self.version==seen:continue
                    seen=self.version;wire=self.wire
                stream.sendall(wire)
                with self.condition:self.sent_frames+=1
        except (OSError,ssl.SSLError,EOFError):pass  # Connection-local; never a simulation fault.
        finally:
            stream.close()
            with self.condition:
                self.clients.discard(raw);self.clients.discard(stream);self.threads.discard(threading.current_thread());self.condition.notify_all()

    def close(self):
        self.stopping.set()
        with self.condition:
            self.condition.notify_all();clients=list(self.clients);threads=list(self.threads)
        if self.listener is not None:self.listener.close()
        for stream in clients:
            try:stream.shutdown(socket.SHUT_RDWR)
            except OSError:pass
            stream.close()
        if self.accept_thread is not None:self.accept_thread.join(3.)
        for thread in threads:thread.join(3.)
        if (self.accept_thread is not None and self.accept_thread.is_alive()) or any(t.is_alive() for t in threads):raise RuntimeError('publisher shutdown deadline exceeded')

    def __enter__(self):self.start();return self
    def __exit__(self,*_):self.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True,type=Path,help='Private JSON: certificate, private_key, token_hex')
    parser.add_argument('--packet',required=True,type=Path,help='One raw complete snapshot; serves it frozen to each client')
    parser.add_argument('--host',default='127.0.0.1',help='Numeric bind address; LAN binding requires this explicit option')
    parser.add_argument('--port',type=int,default=7443)
    args=parser.parse_args();config=load_json(args.config,16384);fields(config,('certificate','private_key','token_hex'))
    if args.packet.stat().st_size>MAX_MESSAGE:raise ValueError('packet file too large')
    with args.packet.open('rb') as stream:raw=stream.read(MAX_MESSAGE+1)
    with LatestPublisher(config['certificate'],config['private_key'],bytes.fromhex(config['token_hex']),args.host,args.port) as server:
        server.publish(raw);print(json.dumps(dict(listening=True,port=server.address[1],source='frozen packet; no live camera connection')))
        try:
            while True:time.sleep(.2)
        except KeyboardInterrupt:pass


if __name__=='__main__':main()
