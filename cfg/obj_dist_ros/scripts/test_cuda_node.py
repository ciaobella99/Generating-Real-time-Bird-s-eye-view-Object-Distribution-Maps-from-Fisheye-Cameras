#!/home/ciao/anaconda3/envs/carlayolo/bin/python
import rospy
import torch

def main():
    rospy.init_node('test_cuda_node')
    # 印出 torch.cuda.is_available()
    avail = torch.cuda.is_available()
    rospy.loginfo(f"[test_cuda_node] torch.cuda.is_available(): {avail}")
    if avail:
        try:
            # 嘗試在 cuda:0 建立一個 tensor
            x = torch.tensor([42.0], device='cuda:0')
            rospy.loginfo(f"[test_cuda_node] 成功在 GPU 上建立 tensor: {x}")
        except Exception as e:
            rospy.logerr(f"[test_cuda_node] 在 GPU 上建立 tensor 失敗: {e}")
    # 保持節點存活
    rospy.spin()

if __name__ == '__main__':
    main()
