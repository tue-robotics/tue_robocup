def synchronized(L):
    def lock_around(f):
        def locked(*a, **k):
            with L:
                return f(*a, **k)
        locked.__name__ = f.__name__
        locked.__doc__ = f.__doc__
        return locked
    return lock_around
