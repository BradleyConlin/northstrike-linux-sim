#!/usr/bin/env python3
import argparse, torch
from pathlib import Path

class ConvBNReLU(torch.nn.Module):
    def __init__(self, c_in, c_out, k=3, s=1):
        super().__init__(); p=k//2
        self.net=torch.nn.Sequential(
            torch.nn.Conv2d(c_in,c_out,k,s,p,bias=False),
            torch.nn.BatchNorm2d(c_out),
            torch.nn.ReLU(inplace=True))
    def forward(self,x): return self.net(x)

class TinyUNet(torch.nn.Module):
    def __init__(self, in_ch=3, base=32):
        super().__init__()
        self.enc1 = torch.nn.Sequential(ConvBNReLU(in_ch, base), ConvBNReLU(base, base))
        self.enc2 = torch.nn.Sequential(torch.nn.MaxPool2d(2), ConvBNReLU(base, base*2), ConvBNReLU(base*2, base*2))
        self.enc3 = torch.nn.Sequential(torch.nn.MaxPool2d(2), ConvBNReLU(base*2, base*4), ConvBNReLU(base*4, base*4))
        self.bot  = torch.nn.Sequential(torch.nn.MaxPool2d(2), ConvBNReLU(base*4, base*8), ConvBNReLU(base*8, base*8))
        self.up3  = torch.nn.ConvTranspose2d(base*8, base*4, 2, 2)
        self.dec3 = torch.nn.Sequential(ConvBNReLU(base*8, base*4), ConvBNReLU(base*4, base*4))
        self.up2  = torch.nn.ConvTranspose2d(base*4, base*2, 2, 2)
        self.dec2 = torch.nn.Sequential(ConvBNReLU(base*4, base*2), ConvBNReLU(base*2, base*2))
        self.up1  = torch.nn.ConvTranspose2d(base*2, base, 2, 2)
        self.dec1 = torch.nn.Sequential(ConvBNReLU(base*2, base), ConvBNReLU(base, base))
        self.out  = torch.nn.Conv2d(base, 1, 1)
    def forward(self,x):
        e1=self.enc1(x); e2=self.enc2(e1); e3=self.enc3(e2); b=self.bot(e3)
        d3=self.dec3(torch.cat([self.up3(b),e3],1))
        d2=self.dec2(torch.cat([self.up2(d3),e2],1))
        d1=self.dec1(torch.cat([self.up1(d2),e1],1))
        return torch.sigmoid(self.out(d1))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--ckpt', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--size', nargs=2, type=int, default=[320,240])
    args=ap.parse_args()

    ck=torch.load(args.ckpt, map_location='cpu')
    model=TinyUNet(); model.load_state_dict(ck['model']); model.eval()
    dummy=torch.randn(1,3,args.size[1],args.size[0], dtype=torch.float32)
    out=Path(args.out); out.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(model, dummy, str(out),
                      input_names=['rgb'], output_names=['depth_n'],
                      dynamic_axes={'rgb':{0:'N'}, 'depth_n':{0:'N'}},
                      opset_version=17)
    print(f"Exported ONNX -> {out}")

if __name__=='__main__': main()
