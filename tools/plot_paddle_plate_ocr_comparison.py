#!/usr/bin/env python3
"""Plot recorded OCR results on identical crop images; no inference."""
import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageOps


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--paddle-report', type=Path, required=True)
    parser.add_argument('--easy-whole-report', type=Path, required=True)
    parser.add_argument('--easy-regions-report', type=Path, required=True)
    parser.add_argument('--font', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a new output filename')
    paddle = json.loads(args.paddle_report.read_text())
    easy_whole = {r['id']:r for r in json.loads(args.easy_whole_report.read_text())['rows']}
    easy_regions = {r['id']:r for r in json.loads(args.easy_regions_report.read_text())['rows']}
    rows = sorted(paddle['rows'], key=lambda r: r['kind']=='texture_control')
    for r in rows:
        if r['crop_sha256'] != easy_whole[r['id']]['crop_sha256'] or r['crop_sha256'] != easy_regions[r['id']]['crop_sha256']:
            raise ValueError('Comparison requires identical encoded input crops')
    camera = [r for r in rows if r['kind']=='recorded_camera']
    correct = sum(r['exact_match'] for r in camera)
    board = Image.new('RGB',(1600,1200),'#f4f7fb')
    draw = ImageDraw.Draw(board)
    def text(x,y,value,size=23,color='#21354d'):
        draw.text((x,y),value,font=ImageFont.truetype(str(args.font),size),fill=color)
    def prediction(x,y,row):
        value=row['raw_text'].strip()
        color='#16724b' if row['exact_match'] else '#a63831'
        size=27
        while draw.textbbox((0,0),value,font=ImageFont.truetype(str(args.font),size))[2]>225 and size>16:
            size-=1
        text(x,y,value,size,color)
        text(x,y+43,'전체 일치' if row['exact_match'] else '불일치',18,color)
    text(35,25,f'PaddleOCR: 이동 후 사진 {len(camera)}장 중 {correct}장, 번호판 전체 판독 일치',33)
    text(35,80,'정답 123가4567  |  같은 PNG crop  |  문자 교정 없이 NFC·공백만 정규화',23)
    text(35,120,'EasyOCR korean_g2 (11/18) · PaddleOCR korean_PP-OCRv5_mobile_rec (11/19)',20,'#63758c')
    for x,label in [(35,'입력'),(250,'동일 번호판 crop'),(650,'EasyOCR 전체 crop'),
                    (895,'EasyOCR 글자 분리'),(1140,'PaddleOCR 전체 crop'),(1420,'Paddle 점수')]:
        text(x,175,label,19)
    titles={'lateral':'바깥쪽 시작 배치','opposite':'반대쪽 대각선',
            **{f'repeat_{i}':f'같은 배치 재방문 {i}' for i in range(1,4)},'control':'원본 텍스처'}
    for i,row in enumerate(rows):
        y=225+i*135
        text(35,y+27,titles[row['case']],22)
        if row['kind']=='texture_control': text(35,y+64,'별도 대조 입력',18,'#63758c')
        im=ImageOps.contain(Image.open(row['crop_image']).convert('RGB'),(365,100))
        board.paste(im,(250+(365-im.width)//2,y+(100-im.height)//2))
        prediction(650,y+18,easy_whole[row['id']])
        prediction(895,y+18,easy_regions[row['id']])
        prediction(1140,y+18,row)
        text(1430,y+29,f"{row['recognition_confidence']:.3f}",26)
        draw.line((35,y+119,1565,y+119),fill='#d3ddea',width=1)
    text(35,1060,'남은 오류: 바깥쪽 시작 배치의 가 → 7. 원본 텍스처와 다른 네 사진은 정확히 읽었습니다.',23)
    text(35,1104,'한 차량·같은 합성 번호판의 소수 표본이며 재방문 3장이 포함됩니다. 일반적인 정확도 80%를 뜻하지 않습니다.',19,'#63758c')
    text(35,1140,'카메라 crop에는 디버그 검출 상자가 있습니다. 점수는 정답 확률이 아니며, 실시간 이동 전후 OCR은 아직 미검증입니다.',18,'#63758c')
    board.save(args.output)


if __name__=='__main__':
    main()
