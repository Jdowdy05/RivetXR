"""Explicit private provisioning; optional short-lived loopback test certificate."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import secrets
import shutil
import ssl
import subprocess

DEFAULT_OPENSSL=Path(shutil.which('openssl') or ('C:/Program Files/Git/usr/bin/openssl.exe' if os.name=='nt' else '/usr/bin/openssl'))


def certificate_pin(certificate):
    text=Path(certificate).read_text(encoding='ascii')
    match=re.search(r'-----BEGIN CERTIFICATE-----.*?-----END CERTIFICATE-----',text,re.S)
    if not match:raise ValueError('PEM leaf certificate required')
    return hashlib.sha256(ssl.PEM_cert_to_DER_cert(match.group())).hexdigest()


def provision(directory,certificate=None,private_key=None,*,test_certificate=False,openssl=DEFAULT_OPENSSL,port=7443,control_port=None):
    if type(port) is not int or not 1<=port<=65535:raise ValueError('invalid client port')
    if control_port is not None and (type(control_port) is not int or not 1<=control_port<=65535 or control_port==port):raise ValueError('control port must be valid and different from scene port')
    root=Path(directory);root.mkdir(parents=True,exist_ok=True)
    if any(root.iterdir()):raise ValueError('provisioning directory must be empty and ignored')
    if test_certificate:
        if certificate is not None or private_key is not None:raise ValueError('choose provided certificate or test certificate')
        certificate=root/'test-cert.pem';private_key=root/'test-key.pem'
        if not Path(openssl).is_file():raise ValueError('operator must provide openssl executable for test certificate generation')
        subprocess.run([str(openssl),'req','-x509','-newkey','rsa:2048','-nodes','-days','1',
            '-subj','/CN=remote-scene-loopback-test','-keyout',str(private_key),'-out',str(certificate)],
            check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,timeout=30)
    elif certificate is None or private_key is None:raise ValueError('operator certificate and private key required')
    certificate=Path(certificate).resolve();private_key=Path(private_key).resolve()
    context=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER);context.load_cert_chain(certificate,private_key)
    if type(port) is not int or not 1<=port<=65535:raise ValueError('invalid client port')
    token=secrets.token_hex(32)
    server=dict(certificate=str(certificate),private_key=str(private_key),token_hex=token)
    client=dict(version=1,host='127.0.0.1',port=port,certificate_sha256=certificate_pin(certificate),token_hex=token)
    configs=[('server.json',server),('remote-scene-client.json',client)]
    if control_port is not None:
        control_token=secrets.token_hex(32)
        if control_token==token:raise ValueError('scene and control credentials must differ')
        control_server=dict(certificate=str(certificate),private_key=str(private_key),token_hex=control_token)
        control_client=dict(client,port=control_port,token_hex=control_token)
        configs.extend((('gimbal-server.json',control_server),('remote-gimbal-client.json',control_client)))
    for name,value in configs:
        path=root/name;path.write_text(json.dumps(value,indent=2),encoding='utf-8')
        try:path.chmod(0o600)
        except OSError:pass
    return server,client


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output-dir',required=True,type=Path)
    parser.add_argument('--certificate',type=Path);parser.add_argument('--private-key',type=Path)
    parser.add_argument('--test-certificate',action='store_true');parser.add_argument('--openssl',type=Path,default=DEFAULT_OPENSSL)
    parser.add_argument('--port',type=int,default=7443)
    parser.add_argument('--control-port',type=int,help='Also provision a distinct simulation-control token and client/server configs')
    args=parser.parse_args()
    provision(args.output_dir,args.certificate,args.private_key,test_certificate=args.test_certificate,openssl=args.openssl,port=args.port,control_port=args.control_port)
    print('Private server/client configuration written; credentials are not logged. Set a numeric LAN host only for an explicitly provisioned deployment.')


if __name__=='__main__':main()
