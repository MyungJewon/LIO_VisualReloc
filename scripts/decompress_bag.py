# lz4 압축 ROS1 bag → 비압축 bag 변환 (Cpp_SLAM BagParser는 비압축 전용).
# ROS 설치 불필요 — rosbags 라이브러리만 사용. 한 번 변환해두면 재사용.
# 사용: python decompress_bag.py <입력.bag> <출력.bag> [토픽들...(생략=전체)]
import sys
import time

from rosbags.rosbag1 import Reader, Writer


def main(src, dst, topics):
    t0 = time.time()
    with Reader(src) as r, Writer(dst) as w:
        conns = {}
        for c in r.connections:
            if topics and c.topic not in topics:
                continue
            msgdef = c.msgdef.data if hasattr(c.msgdef, 'data') else c.msgdef
            conns[c.id] = w.add_connection(
                c.topic, c.msgtype,
                msgdef=msgdef, md5sum=c.digest,
                callerid=c.ext.callerid, latching=c.ext.latching)
        n = 0
        for conn, timestamp, data in r.messages(
                connections=[c for c in r.connections if c.id in conns]):
            w.write(conns[conn.id], timestamp, data)
            n += 1
            if n % 20000 == 0:
                print(f'  {n} msgs ({time.time() - t0:.0f}s)')
    print(f'완료: {n} msgs → {dst} ({time.time() - t0:.0f}s)')


if __name__ == '__main__':
    if len(sys.argv) < 3:
        sys.exit('사용: python decompress_bag.py <입력.bag> <출력.bag> [토픽...]')
    main(sys.argv[1], sys.argv[2], set(sys.argv[3:]))
