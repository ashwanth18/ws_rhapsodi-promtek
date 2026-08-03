"""Ensures `import ingestion...` resolves to the sibling `ingestion/`
package when running pytest from anywhere, by giving pytest's default
rootdir-insertion logic an unambiguous anchor (a conftest.py with no
`__init__.py` next to it) at the directory containing that package.
"""
