---
layout: post
title: "Updating the Doxygen Documentation"
categories: others
date: 2016-11-25 18:00
published: true
---

In the page "Sources" you can find the main two components of the OCRA project, namely `ocra-recipes` and `ocra-wbi-plugins`, i.e. links to both their source code and documentation. Let us now briefly explain how to update the documentation and have the changes reflected online!

The documentation is currently hosted in the `gh-pages` branch of the `ocra-wbi-plugins` repository. This is not meant to be something permanent, but it's so just for historic reasons. Moreover, the generation of the Doxygen documentation is not so elegant. The procedure is as follows:

1. Create a directory called `html-gh-pages` in your local copy of `ocra-wbi-plugins` (don’t worry, this will be ignored by git).
2. Checkout only the `gh-pages` branch.
3. Compile `ocra-recipes` and `ocra-wbi-plugins` with the `CMake` flag `GENERATE_DOCUMENTATION` active. Both projects will copy the generated documentation in the previously created directory.
4. Push your changes to `gh-pages` from within `html-gh-pages` We need a way to automate this process and make it ‘propre’.