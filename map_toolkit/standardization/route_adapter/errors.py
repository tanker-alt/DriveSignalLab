"""Stable machine-readable errors for offline adaptation."""


class AdapterError(ValueError):
    def __init__(self, code, detail):
        self.code = code
        self.detail = detail
        super().__init__('{}: {}'.format(code, detail))
