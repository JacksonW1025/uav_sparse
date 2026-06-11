from setuptools import find_packages, setup


setup(
    name="cadet",
    version="0.1.0",
    description="CADET measurement pipeline for feasible-but-unsafe UAV pilot inputs",
    package_dir={"": "src"},
    packages=find_packages("src"),
    python_requires=">=3.10",
    install_requires=[
        "matplotlib",
        "numpy",
        "pandas",
        "pyarrow",
        "pymavlink",
        "pyulog",
        "pyyaml",
    ],
    extras_require={"test": ["pytest"]},
)
