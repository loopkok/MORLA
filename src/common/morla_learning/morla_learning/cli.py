"""Validate and materialize a frozen training contract without starting training."""
import argparse
import json
from .contracts import TrainingSpec


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("spec")
    parser.add_argument("--output")
    parser.add_argument("--backend-python")
    parser.add_argument("--backend-root")
    parser.add_argument("--run",action="store_true")
    args = parser.parse_args(argv)
    spec = TrainingSpec.from_dict(json.loads(open(args.spec, encoding="utf-8").read()))
    if args.output:
        spec.write_config(args.output)
    if args.backend_python or args.backend_root:
        if not (args.backend_python and args.backend_root): parser.error('both backend paths are required')
        from .backends import BackendEnvironment,run_training
        result=run_training(spec,BackendEnvironment(args.backend_python,args.backend_root,spec.backend_revision),dry_run=not args.run)
        print(json.dumps(result,ensure_ascii=False,indent=2))
        return 0
    print(json.dumps(spec.config(), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
