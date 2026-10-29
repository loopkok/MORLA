"""Explicit read-only SDK/status probe. Never enables or writes registers."""
import argparse
import json
import time
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True);p.add_argument('--device-id',required=True)
    p.add_argument('--slave-id',type=int,default=9);p.add_argument('--baudrate',type=int,default=115200)
    args=p.parse_args()
    from morla_runtime.resource_manager import ResourceManager
    from .adapter import EPGAdapter
    from .protocol import MODEL
    from morla_core.resource_locator import resolve_package_resource
    descriptor=json.loads(resolve_package_resource('morla_epg_driver','descriptor.json').read_text())
    parameters={**descriptor['defaults'],'port':args.port,'slave_id':args.slave_id,'baudrate':args.baudrate}
    instance={'plugin_id':'morla.jodell.epg50_060','backend':'real','joints':descriptor['joints'],
              'parameters':parameters,'deployment_verified':False,'watchdog_verified':False}
    locks=ResourceManager();adapter=EPGAdapter(instance,{'clock':time.monotonic_ns})
    try:
        locks.acquire(['morla:active-system','epg:'+args.device_id])
        adapter.configure({});health=adapter.health(time.monotonic_ns())
        print(json.dumps({'model':MODEL,'read_only':True,'writes_sent':False,'health':health},ensure_ascii=False,indent=2))
        return 0 if health['ready'] else 2
    finally:
        try:adapter.close()
        finally:locks.release()
