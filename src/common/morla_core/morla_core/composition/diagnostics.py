"""Stable, machine-readable compilation errors."""
class CompilationError(ValueError):
    def __init__(self, message, path="", code="INVALID_CONFIGURATION", instances=()):
        self.diagnostics = [{"error_code": code, "path": path, "message": message,
                            "instances": list(instances), "severity": "error", "suggestion": "Correct the referenced configuration field."}]
        super().__init__(f"{path}: {message}")
