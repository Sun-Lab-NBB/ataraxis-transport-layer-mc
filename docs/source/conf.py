# Configuration file for the Sphinx documentation builder.

# -- Project information -----------------------------------------------------
project = 'ataraxis-transport-layer-mc'
# noinspection PyShadowingBuiltins
copyright = '2025, Sun (NeuroAI) lab'
authors = ['Ivan Kondratyev', 'Jasmine Si']
release = '2.0.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'breathe',             # To read doxygen-generated xml files (to parse C++ documentation).
    'sphinx_rtd_theme',    # To format the documentation HTML using ReadTheDocs format.
    'sphinx_rtd_dark_mode' # Enables dark mode for RTD theme.
]

templates_path = ['_templates']
exclude_patterns = []

# Breathe configuration
breathe_projects = {"ataraxis-transport-layer-mc": "./doxygen/xml"}
breathe_default_project = "ataraxis-transport-layer-mc"

# Disables the dark mode by default.
default_dark_mode = False

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'  # Directs sphinx to use RTD theme
