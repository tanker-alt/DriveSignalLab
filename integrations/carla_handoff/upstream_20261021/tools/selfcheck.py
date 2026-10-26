"""Check imports locally; --server also reads the live server without changing it."""
import argparse
import importlib
import json
import platform
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server",action="store_true")
    parser.add_argument("--host",default="localhost")
    parser.add_argument("--port",type=int,default=2000)
    args=parser.parse_args()
    report=dict(python=platform.python_version(),platform=platform.platform(),
                tested_python="3.7.9",tested_carla="0.9.15",checks={},errors=[])
    for module in ("carla","evaluation.api","map_tools"):
        try:
            importlib.import_module(module);report["checks"][module]="ok"
        except Exception as exc:
            report["errors"].append(module+": "+str(exc))
    for module in ("numpy","matplotlib"):
        try:report["checks"][module]=importlib.import_module(module).__version__
        except ImportError:report["checks"][module]="optional: not installed"
    if not report["errors"]:
        import carla
        client=carla.Client(args.host,args.port);client.set_timeout(5)
        report["client_version"]=client.get_client_version()
        if report["client_version"]!="0.9.15":report["errors"].append("Client version differs from tested 0.9.15")
        if args.server:
            try:
                report["server_version"]=client.get_server_version()
                world=client.get_world();report["map"]=world.get_map().name
                report["frame"]=world.get_snapshot().frame
                if report["server_version"]!=report["client_version"]:
                    report["errors"].append("Client/server version mismatch")
            except Exception as exc:report["errors"].append("server: "+str(exc))
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 1 if report["errors"] else 0


if __name__=="__main__":
    sys.exit(main())
