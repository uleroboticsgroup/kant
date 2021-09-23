#!/usr/bin/env python3

import rclpy
from kant_knowledge_base.knowledge_base import KnowledgeBaseNode


def main(args=None):
    rclpy.init(args=args)

    node = KnowledgeBaseNode()

    node.join_spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
