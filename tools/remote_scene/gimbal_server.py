"""Separate pinned-TLS control endpoint; only virtual pan/tilt is supported."""
import hmac
import ipaddress
import socket
import ssl
import threading
import time
from .publisher import recv_exact
from .gimbal_control import GimbalSession


class SimulatedGimbalServer:
    def __init__(self,driver,certificate,private_key,token,host='127.0.0.1',port=0,*,reset_map=lambda:None,map_generation=lambda:1,source_id=1):
        if getattr(driver,'simulation_only',False) is not True:raise ValueError('only simulated gimbal control is available')
        address=ipaddress.ip_address(host)
        if type(port) is not int or not 0<=port<=65535:raise ValueError('invalid gimbal port')
        if not isinstance(token,bytes) or len(token)!=32:raise ValueError('32-byte gimbal token required')
        self.driver=driver;self.token=token;self.host=host;self.port=port
        self.family=socket.AF_INET6 if address.version==6 else socket.AF_INET
        self.context=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER);self.context.minimum_version=ssl.TLSVersion.TLSv1_2
        self.context.load_cert_chain(certificate,private_key)
        # Callbacks must only set/read atomic requests/counters, never build a map
        # or acquire a map-construction lock while the control watchdog is held.
        self.reset_map=reset_map;self.map_generation=map_generation;self.source_id=source_id
        self.gate=threading.RLock();self.stopping=threading.Event();self.listener=None
        self.accept_thread=None;self.tick_thread=None;self.client_thread=None;self.client=None;self.session=None
        self.rejected=0;self.error=''

    @property
    def address(self):
        if self.listener is None:raise RuntimeError('gimbal service is not started')
        return self.listener.getsockname()[:2]

    def start(self):
        if self.listener is not None or self.stopping.is_set():raise RuntimeError('gimbal lifecycle cannot restart')
        s=socket.socket(self.family,socket.SOCK_STREAM);s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        try:s.bind((self.host,self.port));s.listen(1);s.settimeout(.2)
        except Exception:s.close();raise
        self.listener=s
        self.accept_thread=threading.Thread(target=self._accept,name='gimbal-accept',daemon=True)
        self.tick_thread=threading.Thread(target=self._tick,name='gimbal-watchdog',daemon=True)
        self.accept_thread.start();self.tick_thread.start();return self.address

    def _tick(self):
        while not self.stopping.wait(.01):
            try:self.sample()
            except Exception:
                self.error='Simulated gimbal watchdog failed';self.stopping.set()
                with self.gate:
                    self.driver.hold(time.monotonic_ns())
                    if self.client:
                        try:self.client.shutdown(socket.SHUT_RDWR)
                        except OSError:pass
                return

    def sample(self):
        with self.gate:
            now=time.monotonic_ns()
            if self.session:self.session.tick(now)
            else:self.driver.advance(now)
            return self.driver.feedback()

    def _accept(self):
        while not self.stopping.is_set():
            try:s,_=self.listener.accept()
            except socket.timeout:continue
            except OSError:return
            with self.gate:
                if self.client is not None or self.stopping.is_set():s.close();self.rejected+=1;continue
                self.client=s
                self.client_thread=threading.Thread(target=self._serve,args=(s,),name='gimbal-control',daemon=True)
                self.client_thread.start()

    def _serve(self,raw):
        stream=raw
        try:
            raw.settimeout(2.)
            stream=self.context.wrap_socket(raw,server_side=True,do_handshake_on_connect=False)
            with self.gate:self.client=stream
            stream.do_handshake();auth=recv_exact(stream,36)
            if auth[:4]!=b'QGC1' or not hmac.compare_digest(auth[4:],self.token):
                self.rejected+=1;return
            with self.gate:
                now=time.monotonic_ns();self.session=GimbalSession(self.driver,now,reset_map=self.reset_map,map_generation=self.map_generation,source_id=self.source_id)
                reply=self.session.reply(now)
            stream.sendall(reply);stream.settimeout(.1)
            while not self.stopping.is_set():
                try:first=stream.recv(64)
                except socket.timeout:continue
                if not first:break
                # A timeout after a partial command is a framing failure; close.
                command=first+recv_exact(stream,64-len(first))
                with self.gate:reply=self.session.handle(command,time.monotonic_ns())
                stream.sendall(reply)
        except (OSError,EOFError,ValueError,ssl.SSLError):pass
        except Exception:self.error='Simulated gimbal client failed'
        finally:
            with self.gate:
                if self.session:self.session.close(time.monotonic_ns())
                self.session=None
            stream.close()
            with self.gate:
                if self.client is stream or self.client is raw:self.client=None

    def close(self):
        self.stopping.set()
        if self.listener:self.listener.close()
        with self.gate:
            if self.session:self.session.close(time.monotonic_ns())
            else:self.driver.hold(time.monotonic_ns())
            client=self.client
        if client:
            try:client.shutdown(socket.SHUT_RDWR)
            except OSError:pass
            client.close()
        for thread in (self.accept_thread,self.tick_thread,self.client_thread):
            if thread:thread.join(3.)
            if thread and thread.is_alive():raise RuntimeError('gimbal shutdown deadline exceeded')

    def __enter__(self):self.start();return self
    def __exit__(self,*_):self.close()
