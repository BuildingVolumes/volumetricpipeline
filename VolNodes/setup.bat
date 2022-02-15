@echo off

pip install -r requirements.txt
pip install --pre torch torchvision -f https://download.pytorch.org/whl/nightly/cu113/torch_nightly.html

@pause