#!/usr/bin/env python3

'''Build documentation with doxygen and inject customized search code'''

import os
import shutil
import subprocess
import sys

from lxml import etree

XMLTEMPLATE = '<script id="searchdata_xml" type="text/xmldata">{}</script>'


def build():
    ws = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print(ws, __file__)
    builddir = os.path.join(ws, 'docs', '_build')
    if not os.path.exists(builddir):
        os.makedirs(builddir)
    try:
        on_rtd = os.environ.get('READTHEDOCS', None) == 'True'
        if on_rtd:
            artifacts_url = 'https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/jobs/artifacts/{}/download?job=docs'
            rtd_version = os.environ.get('READTHEDOCS_VERSION', None)
            if rtd_version == 'latest':
                artifacts_url = artifacts_url.format('master')
            else:
                raise ValueError('Unknown RTD version: '.format(rtd_version))
            download_cmd = 'wget -O docs.zip {}'.format(url)
            unzip_cmd = 'unzip docs.zip'
            mv_cmd = 'mv -v docs/_build _build'
            subprocess.run(download_cmd, shell=True, cwd=ws, check=True)
            subprocess.run(unzip_cmd, shell=True, cwd=ws, check=True)
            subprocess.run(mv_cmd, shell=True, cwd=ws, check=True)
        else:
            subprocess.run(['doxygen', 'docs/.doxygen/Doxyfile'], cwd=ws, check=True)
    except subprocess.CalledProcessError as err:
        print('\nDoxygen failed to generate documentation: {}\n'.format(err))
        return 1
    shutil.copyfile(
        os.path.join(ws, 'docs', '.doxygen', 'search.js'),
        os.path.join(builddir, 'html', 'search', 'search.js'))
    with open(os.path.join(builddir, 'html', 'search.html'), 'a') as html:
        with open(os.path.join(builddir, 'searchdata.xml'), 'rb') as xml:
            root = etree.fromstring(xml.read())
            pages = root.xpath('/add/doc/field[@name="type" and text() ="page"]/..')
            for page in pages:
                url = page.xpath('field[@name="url"]/text()')[0]
                with open(os.path.join(builddir, 'html', url)) as pagefile:
                    pageroot = etree.HTML(pagefile.read())
                    title = pageroot.xpath('//div[@class="title"]/text()')[0].strip()
                    field = etree.Element('field')
                    field.attrib['name'] = 'title'
                    field.text = title
                    page.append(field)
            html.write(XMLTEMPLATE.format(etree.tostring(root).decode('utf8')))
    print('\nDocumentation has been built in: {}\n'.format(os.path.join(builddir, 'html', 'index.html')))
    return 0


if __name__ == '__main__':
    sys.exit(build())
