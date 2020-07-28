import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="pixelToBinary",
    version="0.0.1",
    author="Akif Gultekin",
    author_email="akif.gultekin99@gmail.com",
    description="A modular python library for creating Binary Images and Binary Movies, easily.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Akif-G/pixelToBinary",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.8.1',
)